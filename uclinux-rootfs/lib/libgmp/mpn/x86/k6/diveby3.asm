# AMD K6 mpn_divexact_by3 -- mpn division by 3, expecting no remainder.
#
# K6: 11.0 cycles/limb


# Copyright (C) 2000 Free Software Foundation, Inc.
#
# This file is part of the GNU MP Library.
#
# The GNU MP Library is free software; you can redistribute it and/or modify
# it under the terms of the GNU Library General Public License as published by
# the Free Software Foundation; either version 2 of the License, or (at your
# option) any later version.
#
# The GNU MP Library is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library General Public
# License for more details.
#
# You should have received a copy of the GNU Library General Public License
# along with the GNU MP Library; see the file COPYING.LIB.  If not, write to
# the Free Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
# MA 02111-1307, USA.


include(`../config.m4')


# mp_limb_t mpn_divexact_by3 (mp_ptr dst, mp_srcptr src, mp_size_t size);
#
# Divide src,size by 3 and store the quotient in dst,size.  If src,size
# isn't exactly divisible by 3 the result in dst,size won't be very useful.
# The return value is 0 if src,size was divisible by 3, or non-zero if not.
#
# Using %esi in the (%esi,%ecx,4) or 0(%esi,%ecx,4) addressing mode doesn't
# lead to vector decoding, unlike plain (%esi) does.

defframe(PARAM_SIZE,12)
defframe(PARAM_SRC, 8)
defframe(PARAM_DST, 4)
deflit(`FRAME',0)

dnl  multiplicative inverse of 3, modulo 2^32
deflit(INVERSE_3, 0xAAAAAAAB)

	.text
	ALIGN(32)

PROLOGUE(mpn_divexact_by3)

	movl	PARAM_SIZE, %ecx
	pushl	%esi		defframe_pushl(SAVE_ESI)

	movl	PARAM_SRC, %esi
	pushl	%edi		defframe_pushl(SAVE_EDI)

	movl	PARAM_DST, %edi
	pushl	%ebx		defframe_pushl(SAVE_EBX)

	leal	(%esi,%ecx,4), %esi
	xorl	%ebx, %ebx

	pushl	$3		defframe_pushl(VAR_THREE)
	leal	(%edi,%ecx,4), %edi

	negl	%ecx


	ALIGN(32)	# need 32 for claimed speed
L(top):
	# eax	scratch, low product
	# ebx	carry limb (0 to 3)
	# ecx	counter, limbs, negative
	# edx	scratch, high product
	# esi	&src[size]
	# edi	&dst[size]
	# ebp
	#
	# The 0(%esi,%ecx,4) pads so the finishup instructions are on a 32
	# byte boundary, saving a couple of cycles (that's a fixed couple,
	# not per loop).

	movl	0(%esi,%ecx,4), %eax
	subl	%ebx, %eax

	setc	%bl

	imull	$INVERSE_3, %eax

	movl	%eax, (%edi,%ecx,4)
	addl	$2, %ecx

	mull	VAR_THREE

	addl	%edx, %ebx
	loop	L(top)


	movl	SAVE_ESI, %esi
	movl	%ebx, %eax

	movl	SAVE_EBX, %ebx

	movl	SAVE_EDI, %edi
	addl	$FRAME, %esp

	ret

EPILOGUE()
