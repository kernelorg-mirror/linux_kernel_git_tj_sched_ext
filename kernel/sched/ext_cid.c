/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BPF extensible scheduler class: Documentation/scheduler/sched-ext.rst
 *
 * Copyright (c) 2026 Meta Platforms, Inc. and affiliates.
 * Copyright (c) 2026 Tejun Heo <tj@kernel.org>
 */
#include <linux/cacheinfo.h>

#include "ext_cid.h"

/*
 * Per-cpu scratch cmask used by scx_call_op_set_cpumask() to synthesize a
 * cmask from a cpumask. Allocated alongside the cid arrays on first enable
 * and never freed. Sized to the full cid space. Caller holds rq lock so
 * this_cpu_ptr is safe.
 */
static struct scx_cmask __percpu *scx_set_cmask_scratch;

s16 *scx_cid_to_cpu_tbl;
s16 *scx_cpu_to_cid_tbl;
struct scx_cid_topo *scx_cid_topo;

#define SCX_CID_TOPO_NEG	(struct scx_cid_topo) {				\
	.core_cid = -1, .core_idx = -1, .llc_cid = -1, .llc_idx = -1,		\
	.node_cid = -1, .node_idx = -1,						\
}

/*
 * Return @cpu's LLC shared_cpu_map. If cacheinfo isn't populated (offline or
 * !present), record @cpu in @fallbacks and return its node mask instead - the
 * worst that can happen is that the cpu's LLC becomes coarser than reality.
 */
static const struct cpumask *cpu_llc_mask(int cpu, struct cpumask *fallbacks)
{
	struct cpu_cacheinfo *ci = get_cpu_cacheinfo(cpu);

	if (!ci || !ci->info_list || !ci->num_leaves) {
		cpumask_set_cpu(cpu, fallbacks);
		return cpumask_of_node(cpu_to_node(cpu));
	}
	return &ci->info_list[ci->num_leaves - 1].shared_cpu_map;
}

/*
 * The cid arrays are sized by num_possible_cpus() / nr_cpu_ids which are fixed
 * at boot, so allocate once on first enable and never free. Callers can
 * dereference these unconditionally as long as scx_root is non-NULL
 * (rcu_assign_pointer publishes scx_root after scx_cid_init() returns - see
 * scx_root_enable()).
 */
static s32 scx_cid_arrays_alloc(void)
{
	u32 npossible = num_possible_cpus();
	size_t scratch_total = sizeof(struct scx_cmask) +
		SCX_CMASK_NR_WORDS(npossible) * sizeof(u64);
	s16 *cid_to_cpu, *cpu_to_cid;
	struct scx_cid_topo *cid_topo;
	struct scx_cmask __percpu *set_cmask_scratch;

	if (scx_cid_to_cpu_tbl)
		return 0;

	cid_to_cpu = kcalloc(npossible, sizeof(*scx_cid_to_cpu_tbl), GFP_KERNEL);
	cpu_to_cid = kcalloc(nr_cpu_ids, sizeof(*scx_cpu_to_cid_tbl), GFP_KERNEL);
	cid_topo = kmalloc_array(npossible, sizeof(*scx_cid_topo), GFP_KERNEL);
	set_cmask_scratch = __alloc_percpu(scratch_total, sizeof(u64));

	if (!cid_to_cpu || !cpu_to_cid || !cid_topo || !set_cmask_scratch) {
		kfree(cid_to_cpu);
		kfree(cpu_to_cid);
		kfree(cid_topo);
		free_percpu(set_cmask_scratch);
		return -ENOMEM;
	}

	scx_cid_to_cpu_tbl = cid_to_cpu;
	scx_cpu_to_cid_tbl = cpu_to_cid;
	scx_cid_topo = cid_topo;
	scx_set_cmask_scratch = set_cmask_scratch;
	return 0;
}

/**
 * scx_cid_init - build the cid mapping
 * @sch: the scx_sched being initialized; used as the scx_error() target
 *
 * See "Topological CPU IDs" in ext_cid.h for the model. Walk online cpus by
 * intersection at each level (parent_scratch & this_level_mask), which keeps
 * containment correct by construction and naturally splits a physical LLC
 * straddling two NUMA nodes into two LLC units. The caller must hold
 * cpus_read_lock.
 */
s32 scx_cid_init(struct scx_sched *sch)
{
	cpumask_var_t to_walk __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	cpumask_var_t node_scratch __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	cpumask_var_t llc_scratch __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	cpumask_var_t core_scratch __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	cpumask_var_t llc_fallback __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	cpumask_var_t online_no_topo __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	u32 next_cid = 0;
	s32 next_node_idx = 0, next_llc_idx = 0, next_core_idx = 0;
	s32 cpu, ret;

	/* s16 keeps the per-cid arrays compact; widen if NR_CPUS ever grows */
	BUILD_BUG_ON(NR_CPUS > S16_MAX);

	lockdep_assert_cpus_held();

	ret = scx_cid_arrays_alloc();
	if (ret)
		return ret;

	if (!zalloc_cpumask_var(&to_walk, GFP_KERNEL) ||
	    !zalloc_cpumask_var(&node_scratch, GFP_KERNEL) ||
	    !zalloc_cpumask_var(&llc_scratch, GFP_KERNEL) ||
	    !zalloc_cpumask_var(&core_scratch, GFP_KERNEL) ||
	    !zalloc_cpumask_var(&llc_fallback, GFP_KERNEL) ||
	    !zalloc_cpumask_var(&online_no_topo, GFP_KERNEL))
		return -ENOMEM;

	/* -1 sentinels for sparse-possible cpu id holes (0 is a valid cid) */
	for (cpu = 0; cpu < nr_cpu_ids; cpu++)
		scx_cpu_to_cid_tbl[cpu] = -1;

	cpumask_copy(to_walk, cpu_online_mask);

	while (!cpumask_empty(to_walk)) {
		s32 next_cpu = cpumask_first(to_walk);
		s32 nid = cpu_to_node(next_cpu);
		s32 node_cid = next_cid;
		s32 node_idx;

		/*
		 * No NUMA info: skip and let the tail loop assign a no-topo
		 * cid. cpumask_of_node(-1) is undefined.
		 */
		if (nid < 0) {
			cpumask_clear_cpu(next_cpu, to_walk);
			continue;
		}

		node_idx = next_node_idx++;

		/* node_scratch = to_walk & this node */
		cpumask_and(node_scratch, to_walk, cpumask_of_node(nid));
		if (WARN_ON_ONCE(!cpumask_test_cpu(next_cpu, node_scratch)))
			return -EINVAL;

		while (!cpumask_empty(node_scratch)) {
			s32 ncpu = cpumask_first(node_scratch);
			const struct cpumask *llc_mask = cpu_llc_mask(ncpu, llc_fallback);
			s32 llc_cid = next_cid;
			s32 llc_idx = next_llc_idx++;

			/* llc_scratch = node_scratch & this llc */
			cpumask_and(llc_scratch, node_scratch, llc_mask);
			if (WARN_ON_ONCE(!cpumask_test_cpu(ncpu, llc_scratch)))
				return -EINVAL;

			while (!cpumask_empty(llc_scratch)) {
				s32 lcpu = cpumask_first(llc_scratch);
				const struct cpumask *sib = topology_sibling_cpumask(lcpu);
				s32 core_cid = next_cid;
				s32 core_idx = next_core_idx++;
				s32 ccpu;

				/* core_scratch = llc_scratch & this core */
				cpumask_and(core_scratch, llc_scratch, sib);
				if (WARN_ON_ONCE(!cpumask_test_cpu(lcpu, core_scratch)))
					return -EINVAL;

				for_each_cpu(ccpu, core_scratch) {
					s32 cid = next_cid++;

					scx_cid_to_cpu_tbl[cid] = ccpu;
					scx_cpu_to_cid_tbl[ccpu] = cid;
					scx_cid_topo[cid] = (struct scx_cid_topo){
						.core_cid = core_cid,
						.core_idx = core_idx,
						.llc_cid = llc_cid,
						.llc_idx = llc_idx,
						.node_cid = node_cid,
						.node_idx = node_idx,
					};

					cpumask_clear_cpu(ccpu, llc_scratch);
					cpumask_clear_cpu(ccpu, node_scratch);
					cpumask_clear_cpu(ccpu, to_walk);
				}
			}
		}
	}

	/*
	 * No-topo section: any possible cpu without a cid - normally just the
	 * not-online ones. Collect any currently-online cpus that land here in
	 * @online_no_topo so we can warn about them at the end.
	 */
	for_each_cpu(cpu, cpu_possible_mask) {
		s32 cid;

		if (__scx_cpu_to_cid(cpu) != -1)
			continue;
		if (cpu_online(cpu))
			cpumask_set_cpu(cpu, online_no_topo);

		cid = next_cid++;
		scx_cid_to_cpu_tbl[cid] = cpu;
		scx_cpu_to_cid_tbl[cpu] = cid;
		scx_cid_topo[cid] = SCX_CID_TOPO_NEG;
	}

	if (!cpumask_empty(llc_fallback))
		pr_warn("scx_cid: cpus without cacheinfo, using node mask as llc: %*pbl\n",
			cpumask_pr_args(llc_fallback));
	if (!cpumask_empty(online_no_topo))
		pr_warn("scx_cid: online cpus with no usable topology: %*pbl\n",
			cpumask_pr_args(online_no_topo));

	return 0;
}

/**
 * scx_build_cmask_from_cpumask - Build a cmask from a kernel cpumask
 * @cpumask: source cpumask
 *
 * Synthesize a cmask covering the full cid space [0, num_possible_cpus())
 * with bits set for cids whose cpu is in @cpumask. Return a pointer to the
 * per-cpu scratch buffer, valid until the next invocation on this cpu.
 * Caller must hold the rq lock so this_cpu_ptr() is stable.
 */
const struct scx_cmask *scx_build_cmask_from_cpumask(const struct cpumask *cpumask)
{
	struct scx_cmask *cmask;
	s32 cpu;

	lockdep_assert_irqs_disabled();

	cmask = this_cpu_ptr(scx_set_cmask_scratch);
	scx_cmask_init(cmask, 0, num_possible_cpus());
	for_each_cpu(cpu, cpumask) {
		s32 cid = __scx_cpu_to_cid(cpu);

		if (cid >= 0)
			__scx_cmask_set(cmask, cid);
	}
	return cmask;
}

__bpf_kfunc_start_defs();

/**
 * scx_bpf_cid_override - Install an explicit cpu->cid mapping
 * @cpu_to_cid: array of nr_cpu_ids s32 entries (cid for each cpu)
 * @cpu_to_cid__sz: must be nr_cpu_ids * sizeof(s32) bytes
 * @aux: implicit BPF argument to access bpf_prog_aux hidden from BPF progs
 *
 * May only be called from ops.init() of the root scheduler. Replace the
 * topology-probed cid mapping with the caller-provided one. Each possible cpu
 * must map to a unique cid in [0, num_possible_cpus()). Topo info is cleared.
 * On invalid input, trigger scx_error() to abort the scheduler.
 */
__bpf_kfunc void scx_bpf_cid_override(const s32 *cpu_to_cid, u32 cpu_to_cid__sz,
				      const struct bpf_prog_aux *aux)
{
	cpumask_var_t seen __free(free_cpumask_var) = CPUMASK_VAR_NULL;
	struct scx_sched *sch;
	bool alloced;
	s32 cpu, cid;

	/* GFP_KERNEL alloc must happen before the rcu read section */
	alloced = zalloc_cpumask_var(&seen, GFP_KERNEL);

	guard(rcu)();

	sch = scx_prog_sched(aux);
	if (unlikely(!sch))
		return;

	if (!alloced) {
		scx_error(sch, "scx_bpf_cid_override: failed to allocate cpumask");
		return;
	}

	if (scx_parent(sch)) {
		scx_error(sch, "scx_bpf_cid_override() only allowed from root sched");
		return;
	}

	if (cpu_to_cid__sz != nr_cpu_ids * sizeof(s32)) {
		scx_error(sch, "scx_bpf_cid_override: expected %zu bytes, got %u",
			  nr_cpu_ids * sizeof(s32), cpu_to_cid__sz);
		return;
	}

	for_each_possible_cpu(cpu) {
		s32 c = cpu_to_cid[cpu];

		if (!cid_valid(sch, c))
			return;
		if (cpumask_test_and_set_cpu(c, seen)) {
			scx_error(sch, "cid %d assigned to multiple cpus", c);
			return;
		}
		scx_cpu_to_cid_tbl[cpu] = c;
		scx_cid_to_cpu_tbl[c] = cpu;
	}

	/* Invalidate stale topo info - the override carries no topology. */
	for (cid = 0; cid < num_possible_cpus(); cid++)
		scx_cid_topo[cid] = SCX_CID_TOPO_NEG;
}

/**
 * scx_bpf_cid_to_cpu - Return the raw CPU id for @cid
 * @cid: cid to look up
 * @aux: implicit BPF argument to access bpf_prog_aux hidden from BPF progs
 *
 * Return the raw CPU id for @cid. Trigger scx_error() and return -EINVAL if
 * @cid is invalid. The cid<->cpu mapping is static for the lifetime of the
 * loaded scheduler, so the BPF side can cache the result to avoid repeated
 * kfunc invocations.
 */
__bpf_kfunc s32 scx_bpf_cid_to_cpu(s32 cid, const struct bpf_prog_aux *aux)
{
	struct scx_sched *sch;

	guard(rcu)();

	sch = scx_prog_sched(aux);
	if (unlikely(!sch))
		return -EINVAL;
	return scx_cid_to_cpu(sch, cid);
}

/**
 * scx_bpf_cpu_to_cid - Return the cid for @cpu
 * @cpu: cpu to look up
 * @aux: implicit BPF argument to access bpf_prog_aux hidden from BPF progs
 *
 * Return the cid for @cpu. Trigger scx_error() and return -EINVAL if @cpu is
 * invalid. The cid<->cpu mapping is static for the lifetime of the loaded
 * scheduler, so the BPF side can cache the result to avoid repeated kfunc
 * invocations.
 */
__bpf_kfunc s32 scx_bpf_cpu_to_cid(s32 cpu, const struct bpf_prog_aux *aux)
{
	struct scx_sched *sch;

	guard(rcu)();

	sch = scx_prog_sched(aux);
	if (unlikely(!sch))
		return -EINVAL;
	return scx_cpu_to_cid(sch, cpu);
}

/**
 * scx_bpf_cid_topo - Copy out per-cid topology info
 * @cid: cid to look up
 * @out__uninit: where to copy the topology info; fully written by this call
 * @aux: implicit BPF argument to access bpf_prog_aux hidden from BPF progs
 *
 * Fill @out__uninit with the topology info for @cid. Trigger scx_error() if
 * @cid is out of range. If @cid is valid but in the no-topo section, all fields
 * are set to -1.
 */
__bpf_kfunc void scx_bpf_cid_topo(s32 cid, struct scx_cid_topo *out__uninit,
				  const struct bpf_prog_aux *aux)
{
	struct scx_sched *sch;

	guard(rcu)();

	sch = scx_prog_sched(aux);
	if (unlikely(!sch) || !cid_valid(sch, cid)) {
		*out__uninit = SCX_CID_TOPO_NEG;
		return;
	}

	*out__uninit = scx_cid_topo[cid];
}

__bpf_kfunc_end_defs();

BTF_KFUNCS_START(scx_kfunc_ids_init)
BTF_ID_FLAGS(func, scx_bpf_cid_override, KF_IMPLICIT_ARGS | KF_SLEEPABLE)
BTF_KFUNCS_END(scx_kfunc_ids_init)

static const struct btf_kfunc_id_set scx_kfunc_set_init = {
	.owner	= THIS_MODULE,
	.set	= &scx_kfunc_ids_init,
	.filter	= scx_kfunc_context_filter,
};

BTF_KFUNCS_START(scx_kfunc_ids_cid)
BTF_ID_FLAGS(func, scx_bpf_cid_to_cpu, KF_IMPLICIT_ARGS)
BTF_ID_FLAGS(func, scx_bpf_cpu_to_cid, KF_IMPLICIT_ARGS)
BTF_ID_FLAGS(func, scx_bpf_cid_topo, KF_IMPLICIT_ARGS)
BTF_KFUNCS_END(scx_kfunc_ids_cid)

static const struct btf_kfunc_id_set scx_kfunc_set_cid = {
	.owner	= THIS_MODULE,
	.set	= &scx_kfunc_ids_cid,
};

/*
 * cmask bulk ops. See ext_cid.h for the layout and semantics: binary ops only
 * touch the intersection of dest and operand ranges; dest bits outside the
 * intersection, and dest head/tail padding, are left untouched. The 64-cid grid
 * alignment of bits[] makes the word-to-word correspondence trivial.
 */
enum {
	CMASK_OP_AND,
	CMASK_OP_OR,
	CMASK_OP_COPY,
};

void scx_cmask_zero(struct scx_cmask *m)
{
	memset(m->bits, 0, SCX_CMASK_NR_WORDS(m->nr_bits) * sizeof(u64));
}

/*
 * Apply @op to one word - dest[@di] = (dest[@di] & ~@mask) | (op(...) & @mask).
 * Only bits in @mask within the word are touched.
 */
static void cmask_op_word(struct scx_cmask *dest, const struct scx_cmask *operand,
			  u32 di, u32 oi, u64 mask, int op)
{
	u64 dv = dest->bits[di];
	u64 ov = operand->bits[oi];
	u64 rv;

	switch (op) {
	case CMASK_OP_AND:
		rv = dv & ov;
		break;
	case CMASK_OP_OR:
		rv = dv | ov;
		break;
	case CMASK_OP_COPY:
		rv = ov;
		break;
	default:
		BUG();
	}

	dest->bits[di] = (dv & ~mask) | (rv & mask);
}

static void cmask_op(struct scx_cmask *dest, const struct scx_cmask *operand, int op)
{
	u32 lo = max(dest->base, operand->base);
	u32 hi = min(dest->base + dest->nr_bits,
		     operand->base + operand->nr_bits);
	u32 d_base = dest->base / 64;
	u32 o_base = operand->base / 64;
	u32 lo_word, hi_word, w;
	u64 head_mask, tail_mask;

	if (lo >= hi)
		return;

	lo_word = lo / 64;
	hi_word = (hi - 1) / 64;
	head_mask = GENMASK_U64(63, lo & 63);
	tail_mask = GENMASK_U64((hi - 1) & 63, 0);

	/* intersection fits in a single word - apply both head and tail */
	if (lo_word == hi_word) {
		cmask_op_word(dest, operand, lo_word - d_base, lo_word - o_base,
			      head_mask & tail_mask, op);
		return;
	}

	/* first word: head mask */
	cmask_op_word(dest, operand, lo_word - d_base, lo_word - o_base, head_mask, op);

	/* interior words: unmasked */
	for (w = lo_word + 1; w < hi_word; w++)
		cmask_op_word(dest, operand, w - d_base, w - o_base,
			      GENMASK_U64(63, 0), op);

	/* last word: tail mask */
	cmask_op_word(dest, operand, hi_word - d_base, hi_word - o_base, tail_mask, op);
}

/*
 * scx_cmask_and/or/copy only modify @dest bits that lie in the intersection
 * of [@dest->base, @dest->base + @dest->nr_bits) and [@operand->base,
 * @operand->base + @operand->nr_bits). Bits in @dest outside that window keep
 * their prior values - in particular, scx_cmask_copy() does NOT zero @dest
 * bits that lie outside @operand's range.
 */
void scx_cmask_and(struct scx_cmask *dest, const struct scx_cmask *operand)
{
	cmask_op(dest, operand, CMASK_OP_AND);
}

void scx_cmask_or(struct scx_cmask *dest, const struct scx_cmask *operand)
{
	cmask_op(dest, operand, CMASK_OP_OR);
}

void scx_cmask_copy(struct scx_cmask *dest, const struct scx_cmask *operand)
{
	cmask_op(dest, operand, CMASK_OP_COPY);
}

/**
 * scx_cmask_next_set - find the first set bit at or after @cid
 * @m: cmask to search
 * @cid: starting cid (clamped to @m->base if below)
 *
 * Returns the smallest set cid in [@cid, @m->base + @m->nr_bits), or
 * @m->base + @m->nr_bits if none (the out-of-range sentinel matches the
 * termination condition used by scx_cmask_for_each_set()).
 */
u32 scx_cmask_next_set(const struct scx_cmask *m, u32 cid)
{
	u32 end = m->base + m->nr_bits;
	u32 base = m->base / 64;
	u32 last_wi = (end - 1) / 64 - base;
	u32 wi;
	u64 word;

	if (cid < m->base)
		cid = m->base;
	if (cid >= end)
		return end;

	wi = cid / 64 - base;
	word = m->bits[wi] & GENMASK_U64(63, cid & 63);

	while (!word) {
		if (++wi > last_wi)
			return end;
		word = m->bits[wi];
	}

	cid = (base + wi) * 64 + __ffs64(word);
	return cid < end ? cid : end;
}

int scx_cid_kfunc_init(void)
{
	return register_btf_kfunc_id_set(BPF_PROG_TYPE_STRUCT_OPS, &scx_kfunc_set_init) ?:
		register_btf_kfunc_id_set(BPF_PROG_TYPE_STRUCT_OPS, &scx_kfunc_set_cid) ?:
		register_btf_kfunc_id_set(BPF_PROG_TYPE_TRACING, &scx_kfunc_set_cid) ?:
		register_btf_kfunc_id_set(BPF_PROG_TYPE_SYSCALL, &scx_kfunc_set_cid);
}
