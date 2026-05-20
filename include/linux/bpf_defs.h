/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Subset of bpf.h declarations, split out so files that need only these
 * declarations can avoid bpf.h's full include cost.
 */
#ifndef _LINUX_BPF_DEFS_H
#define _LINUX_BPF_DEFS_H

bool bpf_arena_handle_page_fault(unsigned long addr, bool is_write, unsigned long fault_ip);

#endif /* _LINUX_BPF_DEFS_H */
