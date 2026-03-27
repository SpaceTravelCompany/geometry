package linalg_ex

import "base:intrinsics"
//import "core:fmt"
import "core:math/fixed"
import "core:math/linalg"
import "core:testing"

BezierKind :: enum u8 {
	Line,
	Quad,
	Cubic,
}

bezier_order :: #force_inline proc "contextless" (kind: BezierKind) -> int {
	switch kind {
	case .Line:
		return 2
	case .Quad:
		return 3
	case .Cubic:
		return 4
	}
	return 0
}

// Polymorphic arithmetic helpers (fixed / float)

@(private = "file")
_bz_add :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a + b} else {return fixed.add(a, b)}
}

@(private = "file")
_bz_sub :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a - b} else {return fixed.sub(a, b)}
}

@(private = "file")
_bz_mul :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a * b} else {return fixed.mul(a, b)}
}

@(private = "file")
_bz_div :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a / b} else {return fixed.div(a, b)}
}

@(private = "file")
_bz_lt :: #force_inline proc "contextless" (
	a, b: $T,
) -> bool where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a < b} else {return a.i < b.i}
}

@(private = "file")
_bz_gt :: #force_inline proc "contextless" (
	a, b: $T,
) -> bool where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a > b} else {return a.i > b.i}
}

@(private = "file")
_bz_le :: #force_inline proc "contextless" (
	a, b: $T,
) -> bool where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a <= b} else {return a.i <= b.i}
}

@(private = "file")
_bz_ge :: #force_inline proc "contextless" (
	a, b: $T,
) -> bool where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a >= b} else {return a.i >= b.i}
}

@(private = "file")
_bz_eq :: #force_inline proc "contextless" (
	a, b: $T,
) -> bool where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {return a == b} else {return a.i == b.i}
}

@(private = "file")
_bz_mid :: #force_inline proc "contextless" (
	a, b: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(
		T,
	) {return (a + b) * T(0.5)} else {return T{i = (a.i + b.i) >> 1}}
}

@(private = "file")
_bz_ratio :: #force_inline proc "contextless" (num, den: int, $T: typeid) -> T {
	when intrinsics.type_is_float(T) {
		return T(num) / T(den)
	} else {
		Frac :: intrinsics.type_polymorphic_record_parameter_value(T, 1)
		Backing :: intrinsics.type_polymorphic_record_parameter_value(T, 0)
		return T{i = (Backing(num) << Frac) / Backing(den)}
	}
}

// Split Bézier at t=0.5, returning left and right halves.
@(private = "file")
_split_half :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
) -> (
	left, right: [4][2]T,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	half :=
		T(0.5) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1) >> 1,
		}
	switch kind {
	case .Line:
		mid := SubdivLine([2][2]T{pts[0], pts[1]}, half)
		left = {pts[0], mid, {}, {}}
		right = {mid, pts[1], {}, {}}
	case .Quad:
		p01, m, p12 := SubdivQuadraticBezier([3][2]T{pts[0], pts[1], pts[2]}, half)
		left = {pts[0], p01, m, {}}
		right = {m, p12, pts[2], {}}
	case .Cubic:
		c0, c1, m, d0, d1 := SubdivCubicBezier(pts, half)
		left = {pts[0], c0, c1, m}
		right = {m, d0, d1, pts[3]}
	}
	return
}


// Signed distance from point to line (unnormalized cross product).
@(private = "file")
_dist_to_baseline :: #force_inline proc "contextless" (
	px, py, ax, ay, dx, dy: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	return _bz_sub(_bz_mul(_bz_sub(px, ax), dy), _bz_mul(_bz_sub(py, ay), dx))
}

// Clip convex hull of distance control polygon against fat line band [d_min, d_max].
// Returns valid parameter range. t_lo > t_hi means no intersection.
@(private = "file")
_clip_hull :: proc "contextless" (
	d: [4]$T,
	n: int,
	d_min, d_max: T,
) -> (
	t_lo, t_hi: T,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	zero := T(0) when intrinsics.type_is_float(T) else T{i = 0}
	t_lo = one
	t_hi = zero

	for i in 0 ..= n {
		if _bz_ge(d[i], d_min) && _bz_le(d[i], d_max) {
			ti := _bz_ratio(i, n, T)
			if _bz_lt(ti, t_lo) do t_lo = ti
			if _bz_gt(ti, t_hi) do t_hi = ti
		}
	}

	for i in 0 ..= n {
		for j in i + 1 ..= n {
			ti := _bz_ratio(i, n, T)
			tj := _bz_ratio(j, n, T)
			dd := _bz_sub(d[j], d[i])
			if _bz_eq(dd, zero) do continue
			dt := _bz_sub(tj, ti)

			if _bz_lt(d[i], d_min) != _bz_lt(d[j], d_min) {
				tc := _bz_add(ti, _bz_mul(_bz_div(_bz_sub(d_min, d[i]), dd), dt))
				if _bz_ge(tc, zero) && _bz_le(tc, one) {
					if _bz_lt(tc, t_lo) do t_lo = tc
					if _bz_gt(tc, t_hi) do t_hi = tc
				}
			}
			if _bz_gt(d[i], d_max) != _bz_gt(d[j], d_max) {
				tc := _bz_add(ti, _bz_mul(_bz_div(_bz_sub(d_max, d[i]), dd), dt))
				if _bz_ge(tc, zero) && _bz_le(tc, one) {
					if _bz_lt(tc, t_lo) do t_lo = tc
					if _bz_gt(tc, t_hi) do t_hi = tc
				}
			}
		}
	}
	return
}

@(private = "file")
_BezClipWork :: struct($T: typeid) {
	a:       [4][2]T,
	b:       [4][2]T,
	kind_a:  BezierKind,
	kind_b:  BezierKind,
	a_lo:    T,
	a_hi:    T,
	b_lo:    T,
	b_hi:    T,
	swapped: bool, // true when a/b correspond to original b/a
}

// Extract sub-curve for parameter range [t_lo, t_hi].
@(private = "file")
_extract_sub :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	t_lo, t_hi: T,
) -> [4][2]T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	zero := T(0) when intrinsics.type_is_float(T) else T{i = 0}
	result := pts

	if _bz_gt(t_lo, zero) {
		switch kind {
		case .Line:
			p := SubdivLine([2][2]T{pts[0], pts[1]}, t_lo)
			result = {p, pts[1], {}, {}}
		case .Quad:
			_, m, p12 := SubdivQuadraticBezier([3][2]T{pts[0], pts[1], pts[2]}, t_lo)
			result = {m, p12, pts[2], {}}
		case .Cubic:
			_, _, m, d0, d1 := SubdivCubicBezier(pts, t_lo)
			result = {m, d0, d1, pts[3]}
		}
	}

	denom := _bz_sub(one, t_lo)
	if _bz_le(denom, zero) do return result
	t_adj := _bz_div(_bz_sub(t_hi, t_lo), denom)
	if _bz_ge(t_adj, one) do return result

	switch kind {
	case .Line:
		p := SubdivLine([2][2]T{result[0], result[1]}, t_adj)
		result[1] = p
	case .Quad:
		p01, m, _ := SubdivQuadraticBezier([3][2]T{result[0], result[1], result[2]}, t_adj)
		result = {result[0], p01, m, {}}
	case .Cubic:
		c0, c1, m, _, _ := SubdivCubicBezier(result, t_adj)
		result = {result[0], c0, c1, m}
	}
	return result
}

// Bézier Clipping intersection (Sederberg-Nishita).
// Returns the first intersection point and each input curve's t, or ok=false. //TODO Check This Code Later. (maybe only worked now.)
GetBezierIntersectPt :: proc "contextless" (
	kind_a: BezierKind,
	a_in: [4][2]$T,
	kind_b: BezierKind,
	b_in: [4][2]T,
) -> (
	ok: bool,
	ip: [2]T,
	t_a: T,
	t_b: T,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	zero := T(0) when intrinsics.type_is_float(T) else T{i = 0}
	eps: T = epsilon(T) when intrinsics.type_is_float(T) else T{}
	four_fifth := _bz_ratio(4, 5, T)

	Work :: _BezClipWork(T)
	stack: [64]Work
	sp := 1
	stack[0] = Work {
		a       = a_in,
		b       = b_in,
		kind_a  = kind_a,
		kind_b  = kind_b,
		a_lo    = zero,
		a_hi    = one,
		b_lo    = zero,
		b_hi    = one,
		swapped = false,
	}

	for sp > 0 {
		sp -= 1
		w := stack[sp]

		a_range := _bz_sub(w.a_hi, w.a_lo)
		b_range := _bz_sub(w.b_hi, w.b_lo)

		if _bz_le(a_range, eps) && _bz_le(b_range, eps) {
			na := bezier_order(w.kind_a) - 1
			ip.x = _bz_mid(w.a[0].x, w.a[na].x)
			ip.y = _bz_mid(w.a[0].y, w.a[na].y)
			if !w.swapped {
				t_a = _bz_mid(w.a_lo, w.a_hi)
				t_b = _bz_mid(w.b_lo, w.b_hi)
			} else {
				t_a = _bz_mid(w.b_lo, w.b_hi)
				t_b = _bz_mid(w.a_lo, w.a_hi)
			}
			return true, ip, t_a, t_b
		}

		na := bezier_order(w.kind_a) - 1
		ax := w.a[0].x
		ay := w.a[0].y
		dx := _bz_sub(w.a[na].x, ax)
		dy := _bz_sub(w.a[na].y, ay)

		if _bz_eq(dx, zero) && _bz_eq(dy, zero) {
			continue
		}

		d_min := zero
		d_max := zero
		for i in 1 ..< na {
			d := _dist_to_baseline(w.a[i].x, w.a[i].y, ax, ay, dx, dy)
			if _bz_lt(d, d_min) do d_min = d
			if _bz_gt(d, d_max) do d_max = d
		}

		nb := bezier_order(w.kind_b) - 1
		db: [4]T
		for i in 0 ..= nb {
			db[i] = _dist_to_baseline(w.b[i].x, w.b[i].y, ax, ay, dx, dy)
		}

		clip_lo, clip_hi := _clip_hull(db, nb, d_min, d_max)
		if _bz_gt(clip_lo, clip_hi) do continue

		if _bz_lt(clip_lo, zero) do clip_lo = zero
		if _bz_gt(clip_hi, one) do clip_hi = one

		b_span := _bz_sub(w.b_hi, w.b_lo)
		new_b_lo := _bz_add(w.b_lo, _bz_mul(clip_lo, b_span))
		new_b_hi := _bz_add(w.b_lo, _bz_mul(clip_hi, b_span))
		b_clipped := _extract_sub(w.kind_b, w.b, clip_lo, clip_hi)

		clip_range := _bz_sub(clip_hi, clip_lo)

		if _bz_gt(clip_range, four_fifth) {
			if sp + 2 > len(stack) do continue
			if _bz_ge(a_range, b_range) {
				a_left, a_right := _split_half(w.kind_a, w.a)
				a_mid := _bz_mid(w.a_lo, w.a_hi)
				stack[sp] = Work {
					a       = b_clipped,
					b       = a_left,
					kind_a  = w.kind_b,
					kind_b  = w.kind_a,
					a_lo    = new_b_lo,
					a_hi    = new_b_hi,
					b_lo    = w.a_lo,
					b_hi    = a_mid,
					swapped = !w.swapped,
				}
				sp += 1
				stack[sp] = Work {
					a       = b_clipped,
					b       = a_right,
					kind_a  = w.kind_b,
					kind_b  = w.kind_a,
					a_lo    = new_b_lo,
					a_hi    = new_b_hi,
					b_lo    = a_mid,
					b_hi    = w.a_hi,
					swapped = !w.swapped,
				}
				sp += 1
			} else {
				b_left, b_right := _split_half(w.kind_b, b_clipped)
				b_mid := _bz_mid(new_b_lo, new_b_hi)
				stack[sp] = Work {
					a       = b_left,
					b       = w.a,
					kind_a  = w.kind_b,
					kind_b  = w.kind_a,
					a_lo    = new_b_lo,
					a_hi    = b_mid,
					b_lo    = w.a_lo,
					b_hi    = w.a_hi,
					swapped = !w.swapped,
				}
				sp += 1
				stack[sp] = Work {
					a       = b_right,
					b       = w.a,
					kind_a  = w.kind_b,
					kind_b  = w.kind_a,
					a_lo    = b_mid,
					a_hi    = new_b_hi,
					b_lo    = w.a_lo,
					b_hi    = w.a_hi,
					swapped = !w.swapped,
				}
				sp += 1
			}
		} else {
			if sp + 1 > len(stack) do continue
			stack[sp] = Work {
				a       = b_clipped,
				b       = w.a,
				kind_a  = w.kind_b,
				kind_b  = w.kind_a,
				a_lo    = new_b_lo,
				a_hi    = new_b_hi,
				b_lo    = w.a_lo,
				b_hi    = w.a_hi,
				swapped = !w.swapped,
			}
			sp += 1
		}
	}
	return false, {}, zero, zero
}


@(test)
test_2quad_curves :: proc(t: ^testing.T) {
	pt0: [4]linalg.Vector2f32 = {{0.0, 0.0}, {1.0, 0.0}, {1.0, -1.0}, {}} //last leave empty

	pt1: [4]linalg.Vector2f32 = {{1.0, 0.0}, {0.0, 0.0}, {0.0, -1.0}, {}} //last leave empty

	ok, pt, t_a, t_b := GetBezierIntersectPt(.Quad, pt0, .Quad, pt1)


	testing.expect_value(t, ok, true)
	testing.expect_value(t, t_a == t_b, true)
	//fmt.println(ok, pt)
}

