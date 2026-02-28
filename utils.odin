package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"

import "core:math/fixed"

@private make_non_zeroed_slice :: #force_inline proc($T:typeid/[]$E, #any_int len: int, allocator := context.allocator, loc := #caller_location) -> (res:T, err: runtime.Allocator_Error) #optional_allocator_error {
    runtime.make_slice_error_loc(loc, len)
    data : []byte
	data, err = runtime.mem_alloc_non_zeroed(size_of(E) * len, align_of(E), allocator, loc)
	if data == nil && size_of(E) != 0 {
		return nil, err
	}
	(^runtime.Raw_Slice)(&res).data = raw_data(data)
	(^runtime.Raw_Slice)(&res).len  = len
	return
}

// digit-by-digit integer sqrt (port of C sqrt_i64) https://github.com/chmike/fpsqrt
@private sqrt_i64 :: #force_inline proc "contextless" (v: i64) -> i64 {
    b := u64(1) << 62
    q: u64 = 0
    r := u64(v)
    for b > r do b >>= 2
    for b > 0 {
        t := q + b
        q >>= 1
        if r >= t {
            r -= t
            q += b
        }
        b >>= 2
    }
    return i64(q)
}

// Flip the sign (positive ↔ negative)
@(private, require_results)
sign :: #force_inline proc "contextless" (v: $T/fixed.Fixed($Backing, $Fraction_Width)) -> (r: T) {
	r.i = -r.i
	return
}

// [2]FixedDef vector difference
@private sub2_fixed :: #force_inline proc "contextless" (a, b: [2]FixedDef) -> [2]FixedDef {
    return {fixed.sub(a.x, b.x), fixed.sub(a.y, b.y)}
}

// squared length (x*x + y*y), fixed-Vector2f32
@private length2_fixed :: #force_inline proc "contextless" (v: [2]FixedDef) -> FixedDef {
    return fixed.add(fixed.mul(v.x, v.x), fixed.mul(v.y, v.y))
}

// `inject_at_elem` injects an element in a dynamic array at a specified index and moves the previous elements after that index "across"
@private
non_zero_inject_at_elem :: proc(array: ^$T/[dynamic]$E, #any_int index: int, #no_broadcast arg: E, loc := #caller_location) -> (ok: bool, err: runtime.Allocator_Error) #no_bounds_check #optional_allocator_error {
	when !ODIN_NO_BOUNDS_CHECK {
		ensure(index >= 0, "Index must be positive.", loc)
	}
	if array == nil {
		return
	}
	n := max(len(array), index)
	m :: 1
	new_size := n + m

	non_zero_resize(array, new_size, loc) or_return
	when size_of(E) != 0 {
		copy(array[index + m:], array[index:])
		array[index] = arg
	}
	ok = true
	return
}

// `inject_at_elems` injects multiple elements in a dynamic array at a specified index and moves the previous elements after that index "across"
@private
non_zero_inject_at_elems :: proc(array: ^$T/[dynamic]$E, #any_int index: int, #no_broadcast args: ..E, loc := #caller_location) -> (ok: bool, err: runtime.Allocator_Error) #no_bounds_check #optional_allocator_error {
	when !ODIN_NO_BOUNDS_CHECK {
		ensure(index >= 0, "Index must be positive.", loc)
	}
	if array == nil {
		return
	}
	if len(args) == 0 {
		ok = true
		return
	}

	n := max(len(array), index)
	m := len(args)
	new_size := n + m

	non_zero_resize(array, new_size, loc) or_return
	when size_of(E) != 0 {
		copy(array[index + m:], array[index:])
		copy(array[index:], args)
	}
	ok = true
	return
}

@(private, require_results)
invent_bool :: #force_inline proc "contextless" (b, check:bool) -> bool {
	return check ? !b : b
}