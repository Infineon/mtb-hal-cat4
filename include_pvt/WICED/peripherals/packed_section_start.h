/*
 * Declare directives for structure packing. No padding will be provided
 * between the members of packed structures, and therefore, there is no
 * guarantee that structure members will be aligned.
 *
 * Declaring packed structures is compiler specific. In order to handle all
 * cases, packed structures should be delared as:
 *
 * #include <packed_section_start.h>
 *
 * typedef CYHAL_PRE_PACKED_STRUCT struct foobar_t {
 *    some_struct_members;
 * } CYHAL_POST_PACKED_STRUCT foobar_t;
 *
 * #include <packed_section_end.h>
 *
 *
 * $Copyright Open Cypress Semiconductor$
 * $Id: packed_section_start.h 514055 2014-11-08 18:42:49Z sudhirbs $
 */


/* Error check - CYHAL_PACKED_STRUCT is defined in packed_section_start.h
 * and undefined in packed_section_end.h. If it is already defined at this
 * point, then there is a missing include of packed_section_end.h.
 */
#ifdef CYHAL_PACKED_STRUCT
    #error "CYHAL_PACKED_STRUCT is already defined!"
#else
    #define CYHAL_PACKED_STRUCT
#endif


#if defined(_MSC_VER)
    /* Disable compiler warning about pragma pack changing alignment. */
    #pragma warning(disable:4103)

    /* The Microsoft compiler uses pragmas for structure packing. Other
     * compilers use structure attribute modifiers. Refer to
     * CYHAL_PRE_PACKED_STRUCT and CYHAL_POST_PACKED_STRUCT defined below.
     */
    #if defined(BWL_DEFAULT_PACKING)
        /* Default structure packing */
        #pragma pack(push, 8)
    #else   /* CYHAL_PACKED_STRUCT */
        #pragma pack(1)
    #endif   /* CYHAL_PACKED_STRUCT */
#endif   /* _MSC_VER */

#if defined(__GNUC__) && defined(EFI)
#pragma pack(push)
#pragma pack(1)
#endif

/* Declare compiler-specific directives for structure packing. */
#if defined(_MSC_VER)
    #define CYHAL_PRE_PACKED_STRUCT
    #define CYHAL_POST_PACKED_STRUCT
#elif defined(__GNUC__) || defined(__lint)
    #define CYHAL_PRE_PACKED_STRUCT
    #define CYHAL_POST_PACKED_STRUCT    __attribute__ ((packed))
#elif defined(__CC_ARM) || defined ( __IAR_SYSTEMS_ICC__ )
    #define CYHAL_PRE_PACKED_STRUCT    __packed
    #define CYHAL_POST_PACKED_STRUCT
#else
    #error "Unknown compiler!"
#endif
