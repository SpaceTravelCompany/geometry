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

MaxBezierIntersections :: 9

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
	return NumSub(NumMul(NumSub(px, ax), dy), NumMul(NumSub(py, ay), dx))
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
		if NumGe(d[i], d_min) && NumLe(d[i], d_max) {
			ti := NumRatio(i, n, T)
			if NumLt(ti, t_lo) do t_lo = ti
			if NumGt(ti, t_hi) do t_hi = ti
		}
	}

	for i in 0 ..= n {
		for j in i + 1 ..= n {
			ti := NumRatio(i, n, T)
			tj := NumRatio(j, n, T)
			dd := NumSub(d[j], d[i])
			if NumEq(dd, zero) do continue
			dt := NumSub(tj, ti)

			if NumLt(d[i], d_min) != NumLt(d[j], d_min) {
				tc := NumAdd(ti, NumMul(NumDiv(NumSub(d_min, d[i]), dd), dt))
				if NumGe(tc, zero) && NumLe(tc, one) {
					if NumLt(tc, t_lo) do t_lo = tc
					if NumGt(tc, t_hi) do t_hi = tc
				}
			}
			if NumGt(d[i], d_max) != NumGt(d[j], d_max) {
				tc := NumAdd(ti, NumMul(NumDiv(NumSub(d_max, d[i]), dd), dt))
				if NumGe(tc, zero) && NumLe(tc, one) {
					if NumLt(tc, t_lo) do t_lo = tc
					if NumGt(tc, t_hi) do t_hi = tc
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

	if NumGt(t_lo, zero) {
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

	denom := NumSub(one, t_lo)
	if NumLe(denom, zero) do return result
	t_adj := NumDiv(NumSub(t_hi, t_lo), denom)
	if NumGe(t_adj, one) do return result

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
// Returns all intersections found (up to fixed capacity), with each input curve's t.
GetBezierIntersectPt :: proc "contextless" (
	kind_a: BezierKind,
	a_in: [4][2]$T,
	kind_b: BezierKind,
	b_in: [4][2]T,
) -> (
	count: int,
	ips: [MaxBezierIntersections][2]T,
	t_as: [MaxBezierIntersections]T,
	t_bs: [MaxBezierIntersections]T,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	zero := T(0) when intrinsics.type_is_float(T) else T{i = 0}
	eps: T = epsilon(T) when intrinsics.type_is_float(T) else T{}
	eps_hit := NumMul(eps, NumConst(10, T)) when intrinsics.type_is_float(T) else zero
	four_fifth := NumRatio(4, 5, T)

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

		a_range := NumSub(w.a_hi, w.a_lo)
		b_range := NumSub(w.b_hi, w.b_lo)

		if NumLe(a_range, eps) && NumLe(b_range, eps) {
			na := bezier_order(w.kind_a) - 1
			ip := [2]T{NumMid(w.a[0].x, w.a[na].x), NumMid(w.a[0].y, w.a[na].y)}
			t_a: T
			t_b: T
			if !w.swapped {
				t_a = NumMid(w.a_lo, w.a_hi)
				t_b = NumMid(w.b_lo, w.b_hi)
			} else {
				t_a = NumMid(w.b_lo, w.b_hi)
				t_b = NumMid(w.a_lo, w.a_hi)
			}

			is_dup := false
			for i in 0 ..< count {
				if NumLe(NumAbs(NumSub(t_as[i], t_a)), eps_hit) &&
				   NumLe(NumAbs(NumSub(t_bs[i], t_b)), eps_hit) {
					is_dup = true
					break
				}
			}
			if !is_dup && count < len(ips) {
				ips[count] = ip
				t_as[count] = t_a
				t_bs[count] = t_b
				count += 1
			}
			continue
		}

		na := bezier_order(w.kind_a) - 1
		ax := w.a[0].x
		ay := w.a[0].y
		dx := NumSub(w.a[na].x, ax)
		dy := NumSub(w.a[na].y, ay)

		if NumEq(dx, zero) && NumEq(dy, zero) {
			continue
		}

		d_min := zero
		d_max := zero
		for i in 1 ..< na {
			d := _dist_to_baseline(w.a[i].x, w.a[i].y, ax, ay, dx, dy)
			if NumLt(d, d_min) do d_min = d
			if NumGt(d, d_max) do d_max = d
		}

		nb := bezier_order(w.kind_b) - 1
		db: [4]T
		for i in 0 ..= nb {
			db[i] = _dist_to_baseline(w.b[i].x, w.b[i].y, ax, ay, dx, dy)
		}

		clip_lo, clip_hi := _clip_hull(db, nb, d_min, d_max)
		if NumGt(clip_lo, clip_hi) do continue

		if NumLt(clip_lo, zero) do clip_lo = zero
		if NumGt(clip_hi, one) do clip_hi = one

		b_span := NumSub(w.b_hi, w.b_lo)
		new_b_lo := NumAdd(w.b_lo, NumMul(clip_lo, b_span))
		new_b_hi := NumAdd(w.b_lo, NumMul(clip_hi, b_span))
		b_clipped := _extract_sub(w.kind_b, w.b, clip_lo, clip_hi)

		clip_range := NumSub(clip_hi, clip_lo)

		if NumGt(clip_range, four_fifth) {
			if sp + 2 > len(stack) do continue
			if NumGe(a_range, b_range) {
				a_left, a_right := _split_half(w.kind_a, w.a)
				a_mid := NumMid(w.a_lo, w.a_hi)
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
				b_mid := NumMid(new_b_lo, new_b_hi)
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
	return count, ips, t_as, t_bs
}

// Finds parameter t on a Bezier curve from a point by binary search.
GetBezierTFromPoint :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	point: [2]T,
) -> (
	t: T,
	ok: bool,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	zero := T(0) when intrinsics.type_is_float(T) else T{i = 0}
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	ten := NumConst(10, T)
	lo := zero
	hi := one
	cmp_eps := NumMul(epsilon(T), ten) when intrinsics.type_is_float(T) else zero

	for i in 0 ..< 64 {
		mid := NumMid(lo, hi)
		p := EvalBezier(kind, pts, mid)
		diff := Vec2Sub(p, point)
		if NumLe(NumAbs(diff[0]), cmp_eps) && NumLe(NumAbs(diff[1]), cmp_eps) {
			return mid, true
		}

		// Use tangent direction to choose the next half interval.
		tangent := EvalBezierTangent(kind, pts, mid)
		if NumGt(Vec2Dot(tangent, diff), zero) {
			hi = mid
		} else {
			lo = mid
		}
	}

	mid := NumMid(lo, hi)
	p := EvalBezier(kind, pts, mid)
	diff := Vec2Sub(p, point)
	if NumLe(NumAbs(diff[0]), cmp_eps) && NumLe(NumAbs(diff[1]), cmp_eps) {
		return mid, true
	}

	return zero, false
}

// Evaluates a point on a quadratic or cubic Bezier curve.
EvalBezier :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	t: T,
) -> [2]T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	u := NumSub(one, t)
	two := NumConst(2, T)
	three := NumConst(3, T)
	#partial switch kind {
	case .Quad:
		p0, p1, p2 := pts[0], pts[1], pts[2]
		uu := NumMul(u, u)
		tt := NumMul(t, t)
		ut2 := NumMul(NumMul(two, u), t)
		return {
			NumAdd(NumAdd(NumMul(uu, p0[0]), NumMul(ut2, p1[0])), NumMul(tt, p2[0])),
			NumAdd(NumAdd(NumMul(uu, p0[1]), NumMul(ut2, p1[1])), NumMul(tt, p2[1])),
		}
	case .Cubic:
		p0, p1, p2, p3 := pts[0], pts[1], pts[2], pts[3]
		uu := NumMul(u, u)
		tt := NumMul(t, t)
		uuu := NumMul(uu, u)
		ttt := NumMul(tt, t)
		uut3 := NumMul(NumMul(three, uu), t)
		utt3 := NumMul(NumMul(three, u), tt)
		return {
			NumAdd(
				NumAdd(NumAdd(NumMul(uuu, p0[0]), NumMul(uut3, p1[0])), NumMul(utt3, p2[0])),
				NumMul(ttt, p3[0]),
			),
			NumAdd(
				NumAdd(NumAdd(NumMul(uuu, p0[1]), NumMul(uut3, p1[1])), NumMul(utt3, p2[1])),
				NumMul(ttt, p3[1]),
			),
		}
	}
	return {}
}

// Evaluates a tangent on a quadratic or cubic Bezier curve.
EvalBezierTangent :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	t: T,
) -> [2]T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	u := NumSub(one, t)
	two := NumConst(2, T)
	three := NumConst(3, T)
	six := NumConst(6, T)
	switch kind {
	case .Quad:
		p0, p1, p2 := pts[0], pts[1], pts[2]
		return {
			NumAdd(
				NumMul(NumMul(two, u), NumSub(p1[0], p0[0])),
				NumMul(NumMul(two, t), NumSub(p2[0], p1[0])),
			),
			NumAdd(
				NumMul(NumMul(two, u), NumSub(p1[1], p0[1])),
				NumMul(NumMul(two, t), NumSub(p2[1], p1[1])),
			),
		}
	case .Cubic:
		p0, p1, p2, p3 := pts[0], pts[1], pts[2], pts[3]
		uu := NumMul(u, u)
		tt := NumMul(t, t)
		ut6 := NumMul(NumMul(six, u), t)
		return {
			NumAdd(
				NumAdd(
					NumMul(NumMul(three, uu), NumSub(p1[0], p0[0])),
					NumMul(ut6, NumSub(p2[0], p1[0])),
				),
				NumMul(NumMul(three, tt), NumSub(p3[0], p2[0])),
			),
			NumAdd(
				NumAdd(
					NumMul(NumMul(three, uu), NumSub(p1[1], p0[1])),
					NumMul(ut6, NumSub(p2[1], p1[1])),
				),
				NumMul(NumMul(three, tt), NumSub(p3[1], p2[1])),
			),
		}
	}
	return {}
}

@(test)
test_2quad_curves :: proc(t: ^testing.T) {
	pt0: [4]linalg.Vector2f32 = {{0.0, 0.0}, {1.0, 0.0}, {1.0, -1.0}, {}} //last leave empty

	pt1: [4]linalg.Vector2f32 = {{1.0, 0.0}, {0.0, 0.0}, {0.0, -1.0}, {}} //last leave empty

	count, _, t_as, t_bs := GetBezierIntersectPt(.Quad, pt0, .Quad, pt1)

	testing.expect_value(t, count == 1, true)
	testing.expect_value(t, t_as[0] == t_bs[0], true)
	//fmt.println(ok, pt)
}

