
#ifndef TRANSFERLISTLIB_H
#define TRANSFERLISTLIB_H
#include <assert.h>
#include <stdbool.h>
// #include <stdint.h>
#include <Library/DebugLib.h>
// #include <stddef.h>
#include <libfdt.h>
//#include <common/ep_info.h>
//#include <lib/utils_def.h>

#define TRANSFER_LIST_SIGNATURE 0x4a0fb10b
#define TRANSFER_LIST_VERSION 0x0001U
#define BIT(nr)         (1UL << (nr))

/*
 * Init value of maximum alignment required by any TE data in the TL
 * specified as a power of two
 */
#define TRANSFER_LIST_INIT_MAX_ALIGN 3U

/* Alignment required by TE header start address, in bytes */
#define TRANSFER_LIST_GRANULE 8U

/*
 * The round_up() macro rounds up a value to the given boundary in a
 * type-agnostic yet type-safe manner. The boundary must be a power of two.
 * In other words, it computes the smallest multiple of boundary which is
 * greater than or equal to value.
 *
 * round_down() is similar but rounds the value down instead.
 */
#define round_boundary(value, boundary)		\
	((__typeof__(value))((boundary) - 1))

#define round_up(value, boundary)		\
	((((value) - 1) | round_boundary(value, boundary)) + 1)

#define round_down(value, boundary)		\
	((value) & ~round_boundary(value, boundary))

/* add operation together with checking whether the operation overflowed
 * The result is '*res',
 * return 0 on success and 1 on overflow
 */
#define add_overflow(a, b, res) __builtin_add_overflow((a), (b), (res))

/*
 * Round up a value to align with a given size and
 * check whether overflow happens.
 * The rounduped value is '*res',
 * return 0 on success and 1 on overflow
 */
#define round_up_overflow(v, size, res) (__extension__({ \
	typeof(res) __res = res; \
	typeof(*(__res)) __roundup_tmp = 0; \
	typeof(v) __roundup_mask = (typeof(v))(size) - 1; \
	\
	add_overflow((v), __roundup_mask, &__roundup_tmp) ? 1 : \
		(void)(*(__res) = __roundup_tmp & ~__roundup_mask), 0; \
}))

/*
 * Add a with b, then round up the result to align with a given size and
 * check whether overflow happens.
 * The rounduped value is '*res',
 * return 0 on success and 1 on overflow
 */
#define add_with_round_up_overflow(a, b, size, res) (__extension__({ \
	typeof(a) __a = (a); \
	typeof(__a) __add_res = 0; \
	\
	add_overflow((__a), (b), &__add_res) ? 1 : \
		round_up_overflow(__add_res, (size), (res)) ? 1 : 0; \
}))

/**
 * Helper macro to ensure a value lies on a given boundary.
 */
#define is_aligned(value, boundary)			\
	(round_up((uintptr_t) value, boundary) ==	\
	 round_down((uintptr_t) value, boundary))

/*
 * Evaluates to 1 if (ptr + inc) overflows, 0 otherwise.
 * Both arguments must be unsigned pointer values (i.e. uintptr_t).
 */
#define check_uptr_overflow(_ptr, _inc)		\
	((_ptr) > (UINTPTR_MAX - (_inc)))

/*
 * Evaluates to 1 if (u32 + inc) overflows, 0 otherwise.
 * Both arguments must be 32-bit unsigned integers (i.e. effectively uint32_t).
 */
#define check_u32_overflow(_u32, _inc) \
	((_u32) > (UINT32_MAX - (_inc)))

/*
 * Version of the register convention used.
 * Set to 1 for both AArch64 and AArch32 according to fw handoff spec v0.9
 */
#define REGISTER_CONVENTION_VERSION_MASK (1 << 24)


#define TL_FLAGS_HAS_CHECKSUM BIT(0)

enum transfer_list_tag_id {
	TL_TAG_EMPTY = 0,
	TL_TAG_FDT = 1,
	TL_TAG_HOB_BLOCK = 2,
	TL_TAG_HOB_LIST = 3,
	TL_TAG_ACPI_TABLE_AGGREGATE = 4,
	TL_TAG_OPTEE_PAGABLE_PART = 0x100,
	TL_TAG_DT_SPMC_MANIFEST = 0x101,
	TL_TAG_EXEC_EP_INFO64 = 0x102,
	TL_TAG_TB_FW_CONFIG = 0x103,
	TL_TAG_SRAM_LAYOUT64 = 0x104,
};

enum transfer_list_ops {
	TL_OPS_NON, /* invalid for any operation */
	TL_OPS_ALL, /* valid for all operations */
	TL_OPS_RO, /* valid for read only */
	TL_OPS_CUS, /* abort or switch to special code to interpret */
};

struct transfer_list_header {
	uint32_t signature;
	uint8_t checksum;
	uint8_t version;
	uint8_t hdr_size;
	uint8_t alignment; /* max alignment of TE data */
	uint32_t size; /* TL header + all TEs */
	uint32_t max_size;
	uint32_t flags;
	uint32_t reserved; /* spare bytes */
	/*
	 * Commented out element used to visualize dynamic part of the
	 * data structure.
	 *
	 * Note that struct transfer_list_entry also is dynamic in size
	 * so the elements can't be indexed directly but instead must be
	 * traversed in order
	 *
	 * struct transfer_list_entry entries[];
	 */
};

struct __attribute__((packed)) transfer_list_entry {
	uint32_t tag_id : 24;
	uint8_t hdr_size;
	uint32_t data_size;
	/*
	 * Commented out element used to visualize dynamic part of the
	 * data structure.
	 *
	 * Note that padding is added at the end of @data to make to reach
	 * a 8-byte boundary.
	 *
	 * uint8_t	data[ROUNDUP(data_size, 8)];
	 */
};

//ASSERT(sizeof(struct transfer_list_entry) == 0x8U);

void transfer_list_dump(struct transfer_list_header *tl);
// entry_point_info_t *
// transfer_list_set_handoff_args(struct transfer_list_header *tl,
// 			       entry_point_info_t *ep_info);
struct transfer_list_header *transfer_list_init(void *addr, size_t max_size);

struct transfer_list_header *
transfer_list_relocate(struct transfer_list_header *tl, void *addr,
		       size_t max_size);
enum transfer_list_ops
transfer_list_check_header(const struct transfer_list_header *tl);

void transfer_list_update_checksum(struct transfer_list_header *tl);
bool transfer_list_verify_checksum(const struct transfer_list_header *tl);

bool transfer_list_set_data_size(struct transfer_list_header *tl,
				 struct transfer_list_entry *entry,
				 uint32_t new_data_size);

void *transfer_list_entry_data(struct transfer_list_entry *entry);
bool transfer_list_rem(struct transfer_list_header *tl,
		       struct transfer_list_entry *entry);

struct transfer_list_entry *transfer_list_add(struct transfer_list_header *tl,
					      uint32_t tag_id,
					      uint32_t data_size,
					      const void *data);

struct transfer_list_entry *
transfer_list_add_with_align(struct transfer_list_header *tl, uint32_t tag_id,
			     uint32_t data_size, const void *data,
			     uint8_t alignment);

struct transfer_list_entry *
transfer_list_next(struct transfer_list_header *tl,
		   struct transfer_list_entry *last);

struct transfer_list_entry *transfer_list_find(struct transfer_list_header *tl,
					       uint32_t tag_id);

#endif /*__TRANSFER_LIST_H*/
