package linalg_ex

import "base:intrinsics"
//import "core:fmt"
import "core:math"
import "core:math/fixed"
import "core:math/linalg"
import "core:testing"

BezierKind :: enum u8 {
	Line,
	Quad,
	Cubic,
}

MaxBezierIntersections :: 9

@(private = "file")
_root_eps :: #force_inline proc "contextless" ($T: typeid) -> T where intrinsics.type_is_float(T) {
	when T == f64 {
		return T(1e-12)
	}
	return T(1e-6)
}

@(private = "file")
_add_unit_root :: proc "contextless" (roots: ^[4]$T, count: ^int, root: T) where intrinsics.type_is_float(T) {
	eps := _root_eps(T)
	if root < -eps || root > 1 + eps || count^ >= len(roots^) do return
	r := math.clamp(root, T(0), T(1))
	for i in 0 ..< count^ {
		if math.abs(roots^[i] - r) <= eps do return
	}
	roots^[count^] = r
	count^ += 1
}

@(private = "file")
_solve_linear_unit :: proc "contextless" (roots: ^[4]$T, count: ^int, b, c: T) where intrinsics.type_is_float(T) {
	eps := _root_eps(T)
	if math.abs(b) <= eps do return
	_add_unit_root(roots, count, -c / b)
}

@(private = "file")
_solve_quadratic_unit :: proc "contextless" (roots: ^[4]$T, count: ^int, a, b, c: T) where intrinsics.type_is_float(T) {
	eps := _root_eps(T)
	if math.abs(a) <= eps {
		_solve_linear_unit(roots, count, b, c)
		return
	}
	disc := b * b - T(4) * a * c
	if disc < -eps do return
	if math.abs(disc) <= eps {
		_add_unit_root(roots, count, -b / (T(2) * a))
		return
	}
	s := math.sqrt(disc)
	_add_unit_root(roots, count, (-b - s) / (T(2) * a))
	_add_unit_root(roots, count, (-b + s) / (T(2) * a))
}

@(private = "file")
_solve_cubic_unit :: proc "contextless" (roots: ^[4]$T, count: ^int, a, b, c, d: T) where intrinsics.type_is_float(T) {
	eps := _root_eps(T)
	if math.abs(a) <= eps {
		_solve_quadratic_unit(roots, count, b, c, d)
		return
	}

	aa := b / a
	bb := c / a
	cc := d / a
	p := bb - aa * aa / T(3)
	q := T(2) * aa * aa * aa / T(27) - aa * bb / T(3) + cc
	disc := q * q / T(4) + p * p * p / T(27)
	shift := aa / T(3)

	if disc > eps {
		s := math.sqrt(disc)
		u := math.cbrt(-q / T(2) + s)
		v := math.cbrt(-q / T(2) - s)
		_add_unit_root(roots, count, u + v - shift)
		return
	}

	if math.abs(disc) <= eps {
		u := math.cbrt(-q / T(2))
		_add_unit_root(roots, count, T(2) * u - shift)
		_add_unit_root(roots, count, -u - shift)
		return
	}

	if p >= 0 do return
	r := T(2) * math.sqrt(-p / T(3))
	arg := (T(3) * q / (T(2) * p)) * math.sqrt(-T(3) / p)
	arg = math.clamp(arg, T(-1), T(1))
	theta := math.acos(arg) / T(3)
	tau := T(math.TAU)
	for k in 0 ..< 3 {
		_add_unit_root(roots, count, r * math.cos(theta - tau * T(k) / T(3)) - shift)
	}
}

@(private = "file")
_eval_any_bezier :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	t: T,
) -> [2]T where intrinsics.type_is_float(T) {
	if kind == .Line {
		u := T(1) - t
		return {pts[0].x * u + pts[1].x * t, pts[0].y * u + pts[1].y * t}
	}
	return EvalBezier(kind, pts, t)
}

@(private = "file")
_line_t_for_point :: proc "contextless" (line: [4][2]$T, p: [2]T) -> T where intrinsics.type_is_float(T) {
	dx := line[1].x - line[0].x
	dy := line[1].y - line[0].y
	den := dx * dx + dy * dy
	if den <= _root_eps(T) do return T(0)
	return ((p.x - line[0].x) * dx + (p.y - line[0].y) * dy) / den
}

@(private = "file")
_add_line_curve_hit :: proc "contextless" (
	line: [4][2]$T,
	curve_kind: BezierKind,
	curve: [4][2]T,
	t_curve: T,
	count: ^int,
	ips: ^[MaxBezierIntersections][2]T,
	t_line_out: ^[MaxBezierIntersections]T,
	t_curve_out: ^[MaxBezierIntersections]T,
) where intrinsics.type_is_float(T) {
	if count^ >= MaxBezierIntersections do return
	eps := _root_eps(T)
	p := _eval_any_bezier(curve_kind, curve, math.clamp(t_curve, T(0), T(1)))
	t_line := _line_t_for_point(line, p)
	if t_line < -eps || t_line > 1 + eps do return
	t_line = math.clamp(t_line, T(0), T(1))
	t_curve_clamped := math.clamp(t_curve, T(0), T(1))
	for i in 0 ..< count^ {
		if math.abs(t_line_out^[i] - t_line) <= eps && math.abs(t_curve_out^[i] - t_curve_clamped) <= eps do return
	}
	ips^[count^] = p
	t_line_out^[count^] = t_line
	t_curve_out^[count^] = t_curve_clamped
	count^ += 1
}

@(private = "file")
_line_curve_intersections :: proc "contextless" (
	line: [4][2]$T,
	curve_kind: BezierKind,
	curve: [4][2]T,
) -> (
	count: int,
	ips: [MaxBezierIntersections][2]T,
	t_line_out: [MaxBezierIntersections]T,
	t_curve_out: [MaxBezierIntersections]T,
) where intrinsics.type_is_float(T) {
	dx := line[1].x - line[0].x
	dy := line[1].y - line[0].y
	if math.abs(dx) <= _root_eps(T) && math.abs(dy) <= _root_eps(T) do return

	dist: [4]T
	n := bezier_order(curve_kind) - 1
	for i in 0 ..= n {
		dist[i] = (curve[i].x - line[0].x) * dy - (curve[i].y - line[0].y) * dx
	}

	roots: [4]T
	root_count := 0
	switch curve_kind {
	case .Line:
		_solve_linear_unit(&roots, &root_count, dist[1] - dist[0], dist[0])
	case .Quad:
		a := dist[0] - T(2) * dist[1] + dist[2]
		b := T(2) * (dist[1] - dist[0])
		c := dist[0]
		_solve_quadratic_unit(&roots, &root_count, a, b, c)
	case .Cubic:
		a := -dist[0] + T(3) * dist[1] - T(3) * dist[2] + dist[3]
		b := T(3) * dist[0] - T(6) * dist[1] + T(3) * dist[2]
		c := -T(3) * dist[0] + T(3) * dist[1]
		d := dist[0]
		_solve_cubic_unit(&roots, &root_count, a, b, c, d)
	}

	for i in 0 ..< root_count {
		_add_line_curve_hit(line, curve_kind, curve, roots[i], &count, &ips, &t_line_out, &t_curve_out)
	}
	return
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
	when intrinsics.type_is_float(T) {
		if kind_a == .Line {
			return _line_curve_intersections(a_in, kind_b, b_in)
		}
		if kind_b == .Line {
			count, ips, t_line, t_curve := _line_curve_intersections(b_in, kind_a, a_in)
			return count, ips, t_curve, t_line
		}
	}

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

	for i in 0 ..< 64 {
		mid := NumMid(lo, hi)
		p := EvalBezier(kind, pts, mid)
		diff := Vec2Sub(p, point)
		if NumEqE(NumAbs(diff[0]), NumAbs(diff[1])) {
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
	if NumEqE(NumAbs(diff[0]), NumAbs(diff[1])) {
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
	case .Line:
		p0, p1 := pts[0], pts[1]
		return {NumSub(p1[0], p0[0]), NumSub(p1[1], p0[1])}
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

//return -1 if failed
GetBezierTForXMonotone :: proc(
	kind: BezierKind,
	pts: [4]$T,
) -> (
	t0: T,
	t1: T,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	if kind == .Quad {
		// dx/dt = A + B·t = 0
		x0, x1, x2 := pts[0], pts[1], pts[2]
		A: T = NumAdd(NumMul(NumConst(-2, T), x0), NumMul(NumConst(2, T), x1))
		B: T = NumMul(NumConst(2, T), NumAdd(NumSub(x0, NumMul(NumConst(2, T), x1)), x2))

		if NumEq(B, NumConst(0, T)) do return NumConst(-1, T), NumConst(-1, T)
		return NumDiv(NumSign(A), B), NumConst(-1, T)
	} else if kind == .Cubic {
		// dx/dt = coef_A·t² + coef_B·t + coef_C = 0 (x component)
		two := NumConst(2, T)
		three := NumConst(3, T)
		six := NumConst(6, T)
		four := NumConst(4, T)
		x0, x1, x2, x3 := pts[0], pts[1], pts[2], pts[3]
		coef_A := NumMul(
			three,
			NumAdd(NumSub(NumMul(three, x1), x0), NumSub(x3, NumMul(three, x2))),
		)
		coef_B := NumMul(six, NumAdd(NumSub(x0, NumMul(two, x1)), x2))
		coef_C := NumMul(three, NumSub(x1, x0))
		zero := NumConst(0, T)
		two_A := NumMul(two, coef_A)

		if NumEq(coef_A, zero) {
			if NumEq(coef_B, zero) do return NumConst(-1, T), NumConst(-1, T)
			return NumDiv(NumSign(coef_C), coef_B), NumConst(-1, T)
		}

		D := NumSub(NumMul(coef_B, coef_B), NumMul(NumMul(four, coef_A), coef_C))
		if NumLt(D, zero) {
			return NumConst(-1, T), NumConst(-1, T)
		} else if NumEq(D, zero) {
			return NumDiv(NumSign(coef_B), two_A), NumConst(-1, T)
		}

		sqrt_D := NumSqrt(D)
		r0 := NumDiv(NumSub(NumSign(coef_B), sqrt_D), two_A)
		r1 := NumDiv(NumAdd(NumSign(coef_B), sqrt_D), two_A)
		return NumMin(r0, r1), NumMax(r0, r1)
	}
	return NumConst(-1, T), NumConst(-1, T)
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

