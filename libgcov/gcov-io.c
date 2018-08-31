/* Test for GCC >= 3.4.4 && <= 4.4.6 */
//#if ( ( __GNUC__ >  3 ) || \
//      ( __GNUC__ == 3 && __GNUC_MINOR__ > 4 )|| \
//      ( __GNUC__ == 3 && __GNUC_MINOR__ == 4 && __GNUC_PATCHLEVEL__ >= 4 ) ) && \
//    ( ( __GNUC__ <  4 ) || \
//      ( __GNUC__ == 4 && __GNUC_MINOR__ < 4 )|| \
//      ( __GNUC__ == 4 && __GNUC_MINOR__ == 4 && __GNUC_PATCHLEVEL__ <= 6 ) )
/*
 * =====================================================================================
 *
 *       Filename:  gcov-io.c
 *
 *    Description:  This is the I/O file for embedded systems
 *
 *        Version:  1.0
 *        Created:  03/04/08 09:51:59
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Aitor Viana Sanchez (avs), aitor.viana.sanchez@esa.int
 *        Company:  European Space Agency (ESA-ESTEC)
 *
 * =====================================================================================
 */

/* File format for coverage information
   Copyright (C) 1996, 1997, 1998, 2000, 2002,
   2003  Free Software Foundation, Inc.
   Contributed by Bob Manson <manson@cygnus.com>.
   Completely remangled by Nathan Sidwell <nathan@codesourcery.com>.

   This file is part of GCC.

   GCC is free software; you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation; either version 2, or (at your option) any later
   version.

   GCC is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   You should have received a copy of the GNU General Public License
   along with GCC; see the file COPYING.  If not, write to the Free
   Software Foundation, 59 Temple Place - Suite 330, Boston, MA
   02111-1307, USA.  */

#include <stdio.h>
#include <stdlib.h> /* for atexit() */
#include <string.h>
#include "gcov-io.h"

/* Routines declared in gcov-io.h.  This file should be #included by
   another source file, after having #included gcov-io.h.  */


/*  This function shall be defined somewhere else   */
//int send_data(unsigned char * buffer, unsigned int size);

/*-----------------------------------------------------------------------------
 *  PRIVATE INTERFACE
 *-----------------------------------------------------------------------------*/

static void gcov_write_block (unsigned);
static gcov_unsigned_t *gcov_write_words (unsigned);
GCOV_LINKAGE int gcov_send (void);
GCOV_LINKAGE int gcov_close(void);
//static const gcov_unsigned_t *gcov_read_words (unsigned);

extern struct gcov_info * gcov_list;
extern gcov_unsigned_t gcov_crc32;

int dev_id = 0;

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  from_file
 *  Description:  This function just return the given parameter
 * =====================================================================================
 */
static inline gcov_unsigned_t from_file (gcov_unsigned_t value)
{
    return value;
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  gcov_version
 *  Description:  This function returns TRUE (1) if the gcov version is the
 *  version expected. The function returns FALSE (0) in any other case.
 * =====================================================================================
 */
static int gcov_version (struct gcov_info *ptr, gcov_unsigned_t version)
{
    if (version != GCOV_VERSION)
    {
        char v[4], e[4];

        GCOV_UNSIGNED2STRING (v, version);
        GCOV_UNSIGNED2STRING (e, GCOV_VERSION);

        printf ("profiling:%s:Version mismatch - expected %.4s got %.4s\n",
                ptr->filename, e, v);

        return 0;
    }
    return 1;
}


/*-----------------------------------------------------------------------------
 *  PUBLIC INTERFACE
 *-----------------------------------------------------------------------------*/

/* Dump the coverage counts. We merge with existing counts when
   possible, to avoid growing the .da files ad infinitum. We use this
   program's checksum to make sure we only accumulate whole program
   statistics to the correct summary. An object file might be embedded
   in two separate programs, and we must keep the two program
   summaries separate.  */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  gcov_exit
 *  Description:  This function dumps the coverage couns. The merging with
 *  existing counts is not done in embedded systems.
 * =====================================================================================
 */
void gcov_exit (void)
{
    struct gcov_info *gi_ptr;
    struct gcov_summary this_program;
    struct gcov_summary all;
    struct gcov_ctr_summary *cs_ptr;
    const struct gcov_ctr_info *ci_ptr;
    unsigned t_ix;
    gcov_unsigned_t c_num;
    unsigned long coreId = 0;

    /* retrieve the id of the CPU the program is running on */
    #ifdef LEON3
    __asm__ __volatile__("rd     %%asr17,%0\n\t"
                         "srl    %0,28,%0" :
                         "=&r" (coreId) : );
    #endif

    printf("_GCOVEXIT_BEGIN_,core%d\n", coreId); /* see also _GCOVEXIT_END_ */

    if(gcov_list == (void*)0x0)
        printf("%s: gcov_list == NULL\n", __func__);

    memset (&all, 0, sizeof (all));
    /* Find the totals for this execution.  */
    memset (&this_program, 0, sizeof (this_program));
    for (gi_ptr = gcov_list; gi_ptr; gi_ptr = gi_ptr->next)
    {

        ci_ptr = gi_ptr->counts;
        for (t_ix = 0; t_ix < GCOV_COUNTERS_SUMMABLE; t_ix++)
        {
            if (!((1 << t_ix) & gi_ptr->ctr_mask))
                continue;

            cs_ptr = &this_program.ctrs[t_ix];
            cs_ptr->num += ci_ptr->num;
            for (c_num = 0; c_num < ci_ptr->num; c_num++)
            {
                cs_ptr->sum_all += ci_ptr->values[c_num];
                if (cs_ptr->run_max < ci_ptr->values[c_num])
                    cs_ptr->run_max = ci_ptr->values[c_num];
            }
            ci_ptr++;
        }
    }
    /* Now merge each file.  */
    for (gi_ptr = gcov_list; gi_ptr; gi_ptr = gi_ptr->next)
    {
//        struct gcov_summary this_object, object;
//        struct gcov_ctr_summary *cs_obj, *cs_tobj, *cs_prg, *cs_tprg, *cs_all;
//        int error = 0;
//        gcov_unsigned_t tag, length;
//        gcov_position_t summary_pos = 0;
        struct gcov_summary program;
        gcov_type *values[GCOV_COUNTERS];
        const struct gcov_fn_info *fi_ptr;
        unsigned fi_stride;
        unsigned c_ix, f_ix, n_counts;

#if 0
        memset (&this_object, 0, sizeof (this_object));
        memset (&object, 0, sizeof (object));


        /* Totals for this object file.  */
        ci_ptr = gi_ptr->counts;
        for (t_ix = 0; t_ix < GCOV_COUNTERS_SUMMABLE; t_ix++)
        {
            if (!((1 << t_ix) & gi_ptr->ctr_mask))
                continue;

            cs_ptr = &this_object.ctrs[t_ix];
            cs_ptr->num += ci_ptr->num;
            for (c_num = 0; c_num < ci_ptr->num; c_num++)
            {
                cs_ptr->sum_all += ci_ptr->values[c_num];
                if (cs_ptr->run_max < ci_ptr->values[c_num])
                    cs_ptr->run_max = ci_ptr->values[c_num];
            }

            ci_ptr++;
        }
#endif

        c_ix = 0;
        for (t_ix = 0; t_ix < GCOV_COUNTERS; t_ix++)
            if ((1 << t_ix) & gi_ptr->ctr_mask)
            {
                values[c_ix] = gi_ptr->counts[c_ix].values;
                c_ix++;
            }

        /* Calculate the function_info stride. This depends on the
           number of counter types being measured.  */
        fi_stride = sizeof (struct gcov_fn_info) + c_ix * sizeof (unsigned);
        if (__alignof__ (struct gcov_fn_info) > sizeof (unsigned))
        {
            fi_stride += __alignof__ (struct gcov_fn_info) - 1;
            fi_stride &= ~(__alignof__ (struct gcov_fn_info) - 1);
        }

        if (!gcov_open (gi_ptr->filename))
        {
            printf ("profiling:%s:Cannot open\n", gi_ptr->filename);
            continue;
        }

#if 0
        tag = gcov_read_unsigned ();
        if (tag)
        {
            /* Merge data from file.  */
            if (tag != GCOV_DATA_MAGIC)
            {
                fprintf (stderr, "profiling:%s:Not a gcov data file\n",
                        gi_ptr->filename);
read_fatal:;
           gcov_close ();
           continue;
            }
            length = gcov_read_unsigned ();
            if (!gcov_version (gi_ptr, length))
                goto read_fatal;

            length = gcov_read_unsigned ();
            if (length != gi_ptr->stamp)
            {
                /* Read from a different compilation. Overwrite the
                   file.  */
                gcov_truncate ();
                goto rewrite;
            }

            /* Merge execution counts for each function.  */
            for (f_ix = 0; f_ix < gi_ptr->n_functions; f_ix++)
            {
                fi_ptr = (const struct gcov_fn_info *)
                    ((const char *) gi_ptr->functions + f_ix * fi_stride);
                tag = gcov_read_unsigned ();
                length = gcov_read_unsigned ();

                /* Check function.  */
                if (tag != GCOV_TAG_FUNCTION
                        || length != GCOV_TAG_FUNCTION_LENGTH
                        || gcov_read_unsigned () != fi_ptr->ident
                        || gcov_read_unsigned () != fi_ptr->checksum)
                {
read_mismatch:;
              fprintf (stderr, "profiling:%s:Merge mismatch for %s\n",
                      gi_ptr->filename,
                      f_ix + 1 ? "function" : "summaries");
              goto read_fatal;
                }

                c_ix = 0;
                for (t_ix = 0; t_ix < GCOV_COUNTERS; t_ix++)
                {
                    gcov_merge_fn merge;

                    if (!((1 << t_ix) & gi_ptr->ctr_mask))
                        continue;

                    n_counts = fi_ptr->n_ctrs[c_ix];
                    merge = gi_ptr->counts[c_ix].merge;

                    tag = gcov_read_unsigned ();
                    length = gcov_read_unsigned ();
                    if (tag != GCOV_TAG_FOR_COUNTER (t_ix)
                            || length != GCOV_TAG_COUNTER_LENGTH (n_counts))
                        goto read_mismatch;
                    (*merge) (values[c_ix], n_counts);
                    values[c_ix] += n_counts;
                    c_ix++;
                }
                if ((error = gcov_is_error ()))
                    goto read_error;
            }

            f_ix = ~0u;
            /* Check program & object summary */
            while (1)
            {
                gcov_position_t base = gcov_position ();
                int is_program;

                tag = gcov_read_unsigned ();
                if (!tag)
                    break;
                length = gcov_read_unsigned ();
                is_program = tag == GCOV_TAG_PROGRAM_SUMMARY;
                if (length != GCOV_TAG_SUMMARY_LENGTH
                        || (!is_program && tag != GCOV_TAG_OBJECT_SUMMARY))
                    goto read_mismatch;
                gcov_read_summary (is_program ? &program : &object);
                if ((error = gcov_is_error ()))
                    goto read_error;
                if (is_program && program.checksum == gcov_crc32)
                {
                    summary_pos = base;
                    goto rewrite;
                }
            }
        }

        if (!gcov_is_eof ())
        {
read_error:;
           fprintf (stderr, error < 0 ? "profiling:%s:Overflow merging\n"
                   : "profiling:%s:Error merging\n", gi_ptr->filename);
           goto read_fatal;
        }
rewrite:;
        gcov_rewrite ();
        if (!summary_pos)
            memset (&program, 0, sizeof (program));
        /* Merge the summaries.  */
        f_ix = ~0u;
        for (t_ix = 0; t_ix < GCOV_COUNTERS_SUMMABLE; t_ix++)
        {
            cs_obj = &object.ctrs[t_ix];
            cs_tobj = &this_object.ctrs[t_ix];
            cs_prg = &program.ctrs[t_ix];
            cs_tprg = &this_program.ctrs[t_ix];
            cs_all = &all.ctrs[t_ix];

            if ((1 << t_ix) & gi_ptr->ctr_mask)
            {
                if (!cs_obj->runs++)
                    cs_obj->num = cs_tobj->num;
                else if (cs_obj->num != cs_tobj->num)
                    goto read_mismatch;
                cs_obj->sum_all += cs_tobj->sum_all;
                if (cs_obj->run_max < cs_tobj->run_max)
                    cs_obj->run_max = cs_tobj->run_max;
                cs_obj->sum_max += cs_tobj->run_max;

                if (!cs_prg->runs++)
                    cs_prg->num = cs_tprg->num;
                else if (cs_prg->num != cs_tprg->num)
                    goto read_mismatch;
                cs_prg->sum_all += cs_tprg->sum_all;
                if (cs_prg->run_max < cs_tprg->run_max)
                    cs_prg->run_max = cs_tprg->run_max;
                cs_prg->sum_max += cs_tprg->run_max;
            }
            else if (cs_obj->num || cs_prg->num)
                goto read_mismatch;

            if (!cs_all->runs && cs_prg->runs)
                memcpy (cs_all, cs_prg, sizeof (*cs_all));
            else if (!all.checksum
                    && (!GCOV_LOCKED || cs_all->runs == cs_prg->runs)
                    && memcmp (cs_all, cs_prg, sizeof (*cs_all)))
            {
                fprintf (stderr, "profiling:%s:Invocation mismatch - some data files may have been removed%s",
                        gi_ptr->filename, GCOV_LOCKED
                        ? "" : " or concurrent update without locking support");
                all.checksum = ~0u;
            }
        }

        c_ix = 0;
        for (t_ix = 0; t_ix < GCOV_COUNTERS; t_ix++)
            if ((1 << t_ix) & gi_ptr->ctr_mask)
            {
                values[c_ix] = gi_ptr->counts[c_ix].values;
                c_ix++;
            }

#endif
        program.checksum = gcov_crc32;

        /* Write out the data.  */
        gcov_write_tag_length (GCOV_DATA_MAGIC, GCOV_VERSION);
        gcov_write_unsigned (gi_ptr->stamp);

        /* Write execution counts for each function.  */
        for (f_ix = 0; f_ix < gi_ptr->n_functions; f_ix++)
        {
            fi_ptr = (const struct gcov_fn_info *)
                ((const char *) gi_ptr->functions + f_ix * fi_stride);

            /* Announce function.  */
            gcov_write_tag_length (GCOV_TAG_FUNCTION, GCOV_TAG_FUNCTION_LENGTH);
            gcov_write_unsigned (fi_ptr->ident);
            gcov_write_unsigned (fi_ptr->checksum);

            c_ix = 0;
            for (t_ix = 0; t_ix < GCOV_COUNTERS; t_ix++)
            {
                gcov_type *c_ptr;

                if (!((1 << t_ix) & gi_ptr->ctr_mask))
                    continue;

                n_counts = fi_ptr->n_ctrs[c_ix];

                gcov_write_tag_length (GCOV_TAG_FOR_COUNTER (t_ix),
                        GCOV_TAG_COUNTER_LENGTH (n_counts));
                c_ptr = values[c_ix];
                while (n_counts--)
                    gcov_write_counter (*c_ptr++);

                values[c_ix] = c_ptr;
                c_ix++;
            }
        }

        /* Object file summary.  */
        //        gcov_write_summary (GCOV_TAG_OBJECT_SUMMARY, &object);

        /* Generate whole program statistics.  */
        //        gcov_seek (summary_pos);
        //        gcov_write_summary (GCOV_TAG_PROGRAM_SUMMARY, &program);
        gcov_send();
        gcov_close();
        /*        if ((error = gcov_send ()))
                  fprintf (stderr, error  < 0 ?
                  "profiling:%s:Overflow writing\n" :
                  "profiling:%s:Error writing\n",
                  gi_ptr->filename);*/
    }

    printf("_GCOVEXIT_END_,core%d\n", coreId);
}


/* Called before fork or exec - write out profile information gathered so
   far and reset it to zero.  This avoids duplication or loss of the
   profile information gathered so far.  */

    void
__gcov_flush (void)
{
    const struct gcov_info *gi_ptr;

    gcov_exit ();
    for (gi_ptr = gcov_list; gi_ptr; gi_ptr = gi_ptr->next)
    {
        unsigned t_ix;
        const struct gcov_ctr_info *ci_ptr;

        for (t_ix = 0, ci_ptr = gi_ptr->counts; t_ix != GCOV_COUNTERS; t_ix++)
            if ((1 << t_ix) & gi_ptr->ctr_mask)
            {
                memset (ci_ptr->values, 0, sizeof (gcov_type) * ci_ptr->num);
                ci_ptr++;
            }
    }
}



/* Open a gcov file. NAME is the name of the file to open and MODE
   indicates whether a new file should be created, or an existing file
   opened for modification. If MODE is >= 0 an existing file will be
   opened, if possible, and if MODE is <= 0, a new file will be
   created. Use MODE=0 to attempt to reopen an existing file and then
   fall back on creating a new one.  Return zero on failure, >0 on
   opening an existing file and <0 on creating a new one.  */
GCOV_LINKAGE int gcov_open(const char *name)
{
    //  gcov_var.start is cleared in the gcov_close function.
    //  If this variable is not cleared...ERROR
    if( gcov_var.start != 0 )
        return 0;

    // Clear everything
    gcov_var.start = 0;
    gcov_var.offset = gcov_var.length = 0;
    gcov_var.overread = -1u;
    gcov_var.error = 0;


    // copy the filename in the gcov_var structure
    strcpy(gcov_var.filename, name);


    // return 1 means everything is OK
    return 1;
}

/* Close the current gcov file. Flushes data to disk. Returns nonzero
   on failure or error flag set.  */

GCOV_LINKAGE int gcov_send (void)
{
    /*printf("%s: file %s\n", __func__, gcov_var.filename);*/
    if (gcov_var.offset)
        gcov_write_block (gcov_var.offset);

    gcov_var.length = 0;
    return gcov_var.error;
}

GCOV_LINKAGE int gcov_close(void)
{
    /*printf("%s: %s\n", __func__, gcov_var.filename);*/
    memset(gcov_var.filename, 0, strlen(gcov_var.filename));

    //  Clear the start variable because will be tested in the gcov_open
    //  function
    gcov_var.start = 0;

    //  Return the error, not sure whether the error is modifed.
    return gcov_var.error;
}

/* Write out the
 current block, if needs be.  */
/*static void gcov_write_block (unsigned size) {
    unsigned int bw = 0;
    unsigned int i;

    printf("_GCOV_,%s,", gcov_var.filename);
    for(i = 0; i < size << 2; i++) {
        char str[5] = {0};

        printf("%02X", ((unsigned char*)(gcov_var.buffer))[i]);
    }
    printf("\n");

    gcov_var.start += size;
    gcov_var.offset -= size;
}*/

static void gcov_write_block (unsigned size) {
    unsigned char *buffer = (unsigned char*) gcov_var.buffer;
    unsigned int i;

    printf("_GCOV_,%s,", gcov_var.filename);
    /* to speed up the printing process, we display bytes 4 by 4 */
    for(i = 0; i < size; i++) {
        printf("%02X%02X%02X%02X", (unsigned int)(buffer[0]),
        		                   (unsigned int)(buffer[1]),
								   (unsigned int)(buffer[2]),
								   (unsigned int)(buffer[3]));

        buffer += sizeof(gcov_unsigned_t);
    }
    printf("\n");

    gcov_var.start += size;
    gcov_var.offset -= size;
}

/* Allocate space to write BYTES bytes to the gcov file. Return a
   pointer to those bytes, or NULL on failure.  */

static gcov_unsigned_t *gcov_write_words (unsigned words) {
    gcov_unsigned_t *result;

    GCOV_CHECK_WRITING ();
    if (gcov_var.offset >= GCOV_BLOCK_SIZE)
    {
        gcov_write_block (GCOV_BLOCK_SIZE);
        if (gcov_var.offset)
        {
            GCOV_CHECK (gcov_var.offset == 1);
            memcpy (gcov_var.buffer, gcov_var.buffer + GCOV_BLOCK_SIZE, 4);
        }
    }
    result = &gcov_var.buffer[gcov_var.offset];
    gcov_var.offset += words;

    return result;
}

/* Write unsigned VALUE to coverage file.  Sets error flag
   appropriately.  */

    GCOV_LINKAGE void
gcov_write_unsigned (gcov_unsigned_t value)
{
    gcov_unsigned_t *buffer = gcov_write_words (1);

    buffer[0] = value;
}

/* Write counter VALUE to coverage file.  Sets error flag
   appropriately.  */

    GCOV_LINKAGE void
gcov_write_counter (gcov_type value)
{
    gcov_unsigned_t *buffer = gcov_write_words (2);

    buffer[0] = (gcov_unsigned_t) value;
    if (sizeof (value) > sizeof (gcov_unsigned_t))
        buffer[1] = (gcov_unsigned_t) (value >> 32);
    else
        buffer[1] = 0;

//    if (value < 0)
//        gcov_var.error = -1;
}

/* Write a tag TAG and length LENGTH.  */

    GCOV_LINKAGE void
gcov_write_tag_length (gcov_unsigned_t tag, gcov_unsigned_t length)
{
    gcov_unsigned_t *buffer = gcov_write_words (2);

    buffer[0] = tag;
    buffer[1] = length;
}

/* Write a summary structure to the gcov file.  Return nonzero on
   overflow.  */

    GCOV_LINKAGE void
gcov_write_summary (gcov_unsigned_t tag, const struct gcov_summary *summary)
{
    unsigned ix;
    const struct gcov_ctr_summary *csum;

    gcov_write_tag_length (tag, GCOV_TAG_SUMMARY_LENGTH);
    gcov_write_unsigned (summary->checksum);
    for (csum = summary->ctrs, ix = GCOV_COUNTERS_SUMMABLE; ix--; csum++)
    {
        gcov_write_unsigned (csum->num);
        gcov_write_unsigned (csum->runs);
        gcov_write_counter (csum->sum_all);
        gcov_write_counter (csum->run_max);
        gcov_write_counter (csum->sum_max);
    }
}

/* Return a pointer to read BYTES bytes from the gcov file. Returns
   NULL on failure (read past EOF).  */
#if 0
    GCOV_LINKAGE const gcov_unsigned_t *
gcov_read_words (unsigned words)
{
    const gcov_unsigned_t *result;
    unsigned excess = gcov_var.length - gcov_var.offset;

    GCOV_CHECK_READING ();
    if (excess < words)
    {
        gcov_var.start += gcov_var.offset;
        if (excess)
        {
            GCOV_CHECK (excess == 1);
            memcpy (gcov_var.buffer, gcov_var.buffer + gcov_var.offset, 4);
        }
        gcov_var.offset = 0;
        gcov_var.length = excess;
        GCOV_CHECK (!gcov_var.length || gcov_var.length == 1);
        excess = GCOV_BLOCK_SIZE;
        excess = fread (gcov_var.buffer + gcov_var.length,
                1, excess << 2, gcov_var.file) >> 2;
        gcov_var.length += excess;
        if (gcov_var.length < words)
        {
            gcov_var.overread += words - gcov_var.length;
            gcov_var.length = 0;
            return 0;
        }
    }
    result = &gcov_var.buffer[gcov_var.offset];
    gcov_var.offset += words;
    return result;
}


/* Read unsigned value from a coverage file. Sets error flag on file
   error, overflow flag on overflow */

    GCOV_LINKAGE gcov_unsigned_t
gcov_read_unsigned (void)
{
    gcov_unsigned_t value;
    const gcov_unsigned_t *buffer = gcov_read_words (1);

    if (!buffer)
        return 0;
    value = from_file (buffer[0]);
    return value;
}

/* Read counter value from a coverage file. Sets error flag on file
   error, overflow flag on overflow */
#endif
    GCOV_LINKAGE gcov_type
gcov_read_counter (void)
{
#if 0
    gcov_type value;
    const gcov_unsigned_t *buffer = gcov_read_words (2);

    if (!buffer)
        return 0;
    value = from_file (buffer[0]);
    if (sizeof (value) > sizeof (gcov_unsigned_t))
        value |= ((gcov_type) from_file (buffer[1])) << 32;
    else if (buffer[1])
        gcov_var.error = -1;

    if (value < 0)
        gcov_var.error = -1;
    return value;
#endif
    return 0;
}
#if 0

/* Read string from coverage file. Returns a pointer to a static
   buffer, or NULL on empty string. You must copy the string before
   calling another gcov function.  */

    GCOV_LINKAGE void
gcov_read_summary (struct gcov_summary *summary)
{
    unsigned ix;
    struct gcov_ctr_summary *csum;

    summary->checksum = gcov_read_unsigned ();
    for (csum = summary->ctrs, ix = GCOV_COUNTERS_SUMMABLE; ix--; csum++)
    {
        csum->num = gcov_read_unsigned ();
        csum->runs = gcov_read_unsigned ();
        csum->sum_all = gcov_read_counter ();
        csum->run_max = gcov_read_counter ();
        csum->sum_max = gcov_read_counter ();
    }
}

/* Move to the a set position in a gcov file.  BASE is zero to move to
   the end, and nonzero to move to that position.  */

    GCOV_LINKAGE void
gcov_seek (gcov_position_t base)
{
    GCOV_CHECK_WRITING ();
    if (gcov_var.offset)
        gcov_write_block (gcov_var.offset);
    fseek (gcov_var.file, base << 2, base ? SEEK_SET : SEEK_END);
    gcov_var.start = ftell (gcov_var.file) >> 2;
}

#endif

/* Add a new object file onto the bb chain.  Invoked automatically
   when running an object file's global ctors.  */

    void
__gcov_init (struct gcov_info *info)
{
    if (!info->version)
        return;
    if (gcov_version (info, info->version))
    {
        const char *ptr = info->filename;
        gcov_unsigned_t crc32 = gcov_crc32;

        /* Added by LESIA*/
        /*printf("Covered file: %s\n", info->filename);*/
        /* End of Added by LESIA*/

        do
        {
            unsigned ix;
            gcov_unsigned_t value = *ptr << 24;

            for (ix = 8; ix--; value <<= 1)
            {
                gcov_unsigned_t feedback;

                feedback = (value ^ crc32) & 0x80000000 ? 0x04c11db7 : 0;
                crc32 <<= 1;
                crc32 ^= feedback;
            }
        }
        while (*ptr++);

        gcov_crc32 = crc32;

                if (!gcov_list)
                    atexit (gcov_exit);

        info->next = gcov_list;
        gcov_list = info;
    }
    else
        printf("%s: Version mismatch\n", "WARNING");
    info->version = 0;
}
//#endif /* __GNUC__ __GNUC_MINOR__ __GNUC_PATCHLEVEL__ */
