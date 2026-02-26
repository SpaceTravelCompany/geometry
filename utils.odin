package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"

import "core:math/fixed"

@private make_non_zeroed_slice :: proc($T:typeid/[]$E, #any_int len: int, allocator := context.allocator, loc := #caller_location) -> (res:T, err: runtime.Allocator_Error) #optional_allocator_error {
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
@private sqrt_i64 :: proc "contextless" (v: i64) -> i64 {
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
sign :: proc "contextless" (v: $T/fixed.Fixed($Backing, $Fraction_Width)) -> (r: T) {
	r.i = -r.i
	return
}

// [2]FixedDef vector difference
@private sub2_fixed :: proc "contextless" (a, b: [2]FixedDef) -> [2]FixedDef {
    return {fixed.sub(a.x, b.x), fixed.sub(a.y, b.y)}
}

// squared length (x*x + y*y), fixed-Vector2f32
@private length2_fixed :: proc "contextless" (v: [2]FixedDef) -> FixedDef {
    return fixed.add(fixed.mul(v.x, v.x), fixed.mul(v.y, v.y))
}