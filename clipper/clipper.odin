package clipper

import "../linalg_ex"
import "base:intrinsics"
import "base:runtime"
import "core:math"
import "shared:utils_private"

__Clipper_Error :: enum {
	FAILED,
	CANT_FIRST_CURVE,
	OPEN_CURVE_END,
	TOO_SMALL,
	LENGTH_MISMATCH,
}

Clipper_Error :: union #shared_nil {
	__Clipper_Error,
	runtime.Allocator_Error,
}

FillRule :: enum u8 {
	EvenOdd,
	NonZero,
	Positive,
	Negative,
}

ClipType :: enum u8 {
	Union,
	Intersection,
	Difference,
	Xor,
}

@(private = "file")
PathType :: enum u8 {
	Subject,
	Clip,
}

@(private = "file")
Segment :: struct {
	pts:       [4][2]f64,
	kind:      linalg_ex.BezierKind,
	path_type: PathType,
	is_open:   bool,
	path_idx:  int,
	seg_idx:   int,
	order:     int,
}

@(private = "file")
Piece :: struct {
	pts:       [4][2]f64,
	kind:      linalg_ex.BezierKind,
	path_type: PathType,
	is_open:   bool,
	path_idx:  int,
	order:     int,
	used:      bool,
}

@(private = "file")
ParamList :: [dynamic]f64

@(private = "file")
EPS :: 1e-9

@(private = "file")
SIDE_EPS :: 1e-7

@(private = "file")
_end_index :: #force_inline proc "contextless" (kind: linalg_ex.BezierKind) -> int {
	switch kind {
	case .Line:
		return 1
	case .Quad:
		return 2
	case .Cubic:
		return 3
	}
	return 1
}

@(private = "file")
_pt_eq :: #force_inline proc "contextless" (a, b: [2]f64) -> bool {
	return math.abs(a.x - b.x) <= EPS && math.abs(a.y - b.y) <= EPS
}

@(private = "file")
_pt_add :: #force_inline proc "contextless" (a, b: [2]f64) -> [2]f64 {
	return {a.x + b.x, a.y + b.y}
}

@(private = "file")
_pt_sub :: #force_inline proc "contextless" (a, b: [2]f64) -> [2]f64 {
	return {a.x - b.x, a.y - b.y}
}

@(private = "file")
_pt_mul :: #force_inline proc "contextless" (a: [2]f64, s: f64) -> [2]f64 {
	return {a.x * s, a.y * s}
}

@(private = "file")
_cross :: #force_inline proc "contextless" (a, b: [2]f64) -> f64 {
	return a.x * b.y - a.y * b.x
}

@(private = "file")
_dot :: #force_inline proc "contextless" (a, b: [2]f64) -> f64 {
	return a.x * b.x + a.y * b.y
}

@(private = "file")
_seg_start :: #force_inline proc "contextless" (p: Piece) -> [2]f64 {
	return p.pts[0]
}

@(private = "file")
_seg_end_piece :: #force_inline proc "contextless" (p: Piece) -> [2]f64 {
	return p.pts[_end_index(p.kind)]
}

@(private = "file")
_seg_end :: #force_inline proc "contextless" (s: Segment) -> [2]f64 {
	return s.pts[_end_index(s.kind)]
}

@(private = "file")
_eval :: proc "contextless" (kind: linalg_ex.BezierKind, pts: [4][2]f64, t: f64) -> [2]f64 {
	if kind == .Line {
		u := 1.0 - t
		return {pts[0].x * u + pts[1].x * t, pts[0].y * u + pts[1].y * t}
	}
	return linalg_ex.EvalBezier(kind, pts, t)
}

@(private = "file")
_tangent :: proc "contextless" (kind: linalg_ex.BezierKind, pts: [4][2]f64, t: f64) -> [2]f64 {
	if kind == .Line {
		return _pt_sub(pts[1], pts[0])
	}
	return linalg_ex.EvalBezierTangent(kind, pts, t)
}

@(private = "file")
_extract_sub :: proc "contextless" (
	kind: linalg_ex.BezierKind,
	pts: [4][2]f64,
	t0, t1: f64,
) -> [4][2]f64 {
	result := pts
	if t0 > EPS {
		switch kind {
		case .Line:
			p := linalg_ex.SubdivLine([2][2]f64{pts[0], pts[1]}, t0)
			result = {p, pts[1], {}, {}}
		case .Quad:
			_, m, p12 := linalg_ex.SubdivQuadraticBezier([3][2]f64{pts[0], pts[1], pts[2]}, t0)
			result = {m, p12, pts[2], {}}
		case .Cubic:
			_, _, m, d0, d1 := linalg_ex.SubdivCubicBezier(pts, t0)
			result = {m, d0, d1, pts[3]}
		}
	}

	denom := 1.0 - t0
	if denom <= EPS do return result
	t_adj := (t1 - t0) / denom
	if t_adj >= 1.0 - EPS do return result

	switch kind {
	case .Line:
		p := linalg_ex.SubdivLine([2][2]f64{result[0], result[1]}, t_adj)
		result[1] = p
	case .Quad:
		p01, m, _ := linalg_ex.SubdivQuadraticBezier([3][2]f64{result[0], result[1], result[2]}, t_adj)
		result = {result[0], p01, m, {}}
	case .Cubic:
		c0, c1, m, _, _ := linalg_ex.SubdivCubicBezier(result, t_adj)
		result = {result[0], c0, c1, m}
	}
	return result
}

@(private = "file")
_reverse_piece :: proc "contextless" (p: Piece) -> Piece {
	res := p
	switch p.kind {
	case .Line:
		res.pts = {p.pts[1], p.pts[0], {}, {}}
	case .Quad:
		res.pts = {p.pts[2], p.pts[1], p.pts[0], {}}
	case .Cubic:
		res.pts = {p.pts[3], p.pts[2], p.pts[1], p.pts[0]}
	}
	return res
}

@(private = "file")
_append_param :: proc(list: ^ParamList, t: f64) -> Clipper_Error {
	if t < -EPS || t > 1.0 + EPS do return nil
	tt := math.clamp(t, 0.0, 1.0)
	for v in list[:] {
		if math.abs(v - tt) <= EPS do return nil
	}
	non_zero_append(list, tt) or_return
	return nil
}

@(private = "file")
_sort_params :: proc(list: ^ParamList) {
	for i in 1 ..< len(list) {
		j := i
		for j > 0 && list[j - 1] > list[j] {
			list[j - 1], list[j] = list[j], list[j - 1]
			j -= 1
		}
	}
}

@(private = "file")
_append_x_monotone_params :: proc(list: ^ParamList, kind: linalg_ex.BezierKind, pts: [4][2]f64) -> Clipper_Error {
	if kind == .Line do return nil
	t0, t1 := linalg_ex.GetBezierTForXMonotone(kind, [4]f64{pts[0].x, pts[1].x, pts[2].x, pts[3].x})
	_append_param(list, t0) or_return
	_append_param(list, t1) or_return
	return nil
}

@(private = "file")
_append_segment_from_params :: proc(
	out: ^[dynamic]Segment,
	seg: Segment,
	params: ParamList,
) -> Clipper_Error {
	params_sorted := params
	_sort_params(&params_sorted)
	for i in 0 ..< len(params_sorted) - 1 {
		t0 := params_sorted[i]
		t1 := params_sorted[i + 1]
		if t1 - t0 <= EPS do continue
		s := seg
		s.pts = _extract_sub(seg.kind, seg.pts, t0, t1)
		non_zero_append(out, s) or_return
	}
	return nil
}

@(private = "file")
_append_raw_segment :: proc(
	out: ^[dynamic]Segment,
	seg: Segment,
) -> Clipper_Error {
	params := make(ParamList, context.temp_allocator) or_return
	_append_param(&params, 0.0) or_return
	_append_param(&params, 1.0) or_return
	_append_x_monotone_params(&params, seg.kind, seg.pts) or_return
	_append_segment_from_params(out, seg, params) or_return
	return nil
}

@(private = "file")
_append_path_segments :: proc(
	out: ^[dynamic]Segment,
	paths: [][][2]f64,
	is_curves: [][]bool,
	path_type: PathType,
	is_open: bool,
	order: ^int,
) -> Clipper_Error {
	for path, path_idx in paths {
		if is_open {
			if len(path) < 2 do return .TOO_SMALL
		} else {
			if len(path) < 3 do return .TOO_SMALL
		}

		curves: []bool
		if is_curves != nil {
			if path_idx >= len(is_curves) || len(is_curves[path_idx]) != len(path) do return .LENGTH_MISMATCH
			curves = is_curves[path_idx]
			if len(curves) > 0 && curves[0] do return .CANT_FIRST_CURVE
		}

		j := 0
		seg_idx := 0
		for j < len(path) {
			if is_open && j >= len(path) - 1 do break

			seg := Segment {
				kind      = .Line,
				path_type = path_type,
				is_open   = is_open,
				path_idx  = path_idx,
				seg_idx   = seg_idx,
				order     = order^,
			}
			seg.pts[0] = path[j]

			next := j + 1
			if next >= len(path) {
				if is_open do break
				next = 0
			}

			if curves != nil && curves[next] {
				seg.pts[1] = path[next]
				next2 := next + 1
				if next2 >= len(path) {
					if is_open do return .OPEN_CURVE_END
					next2 = 0
				}
				if curves[next2] {
					seg.pts[2] = path[next2]
					next3 := next2 + 1
					if next3 >= len(path) {
						if is_open do return .OPEN_CURVE_END
						next3 = 0
					}
					if curves[next3] do return .OPEN_CURVE_END
					seg.pts[3] = path[next3]
					seg.kind = .Cubic
					j += 3
				} else {
					seg.pts[2] = path[next2]
					seg.kind = .Quad
					j += 2
				}
			} else {
				seg.pts[1] = path[next]
				j += 1
			}

			if !is_open && j >= len(path) do j = len(path)
			_append_raw_segment(out, seg) or_return
			order^ += 1
			seg_idx += 1
		}
	}
	return nil
}

@(private = "file")
_to_f64_paths :: proc(paths: [][][2]$T) -> (out: [][][2]f64, err: Clipper_Error) where intrinsics.type_is_float(T) {
	if paths == nil do return nil, nil
	out = make([][][2]f64, len(paths), context.temp_allocator) or_return
	for path, i in paths {
		out[i] = make([][2]f64, len(path), context.temp_allocator) or_return
		for p, j in path {
			out[i][j] = {f64(p.x), f64(p.y)}
		}
	}
	return
}

@(private = "file")
_from_f64_paths :: proc(
	$T: typeid,
	paths: [][][2]f64,
	allocator := context.allocator,
) -> (
	out: [][][2]T,
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	if paths == nil do return nil, nil
	out = utils_private.make_non_zeroed_slice([][][2]T, len(paths), allocator) or_return
	for path, i in paths {
		out[i] = utils_private.make_non_zeroed_slice([][2]T, len(path), allocator) or_return
		for p, j in path {
			out[i][j] = {T(p.x), T(p.y)}
		}
	}
	return
}

@(private = "file")
_copy_bool_paths :: proc(paths: [][]bool, allocator := context.allocator) -> (out: [][]bool, err: Clipper_Error) {
	if paths == nil do return nil, nil
	out = utils_private.make_non_zeroed_slice([][]bool, len(paths), allocator) or_return
	for path, i in paths {
		out[i] = utils_private.make_non_zeroed_slice([]bool, len(path), allocator) or_return
		for v, j in path do out[i][j] = v
	}
	return
}

@(private = "file")
_line_intersection_params :: proc "contextless" (
	a0, a1, b0, b1: [2]f64,
) -> (
	kind: linalg_ex.IntersectKind,
	ta, tb: f64,
) {
	r := _pt_sub(a1, a0)
	s := _pt_sub(b1, b0)
	den := _cross(r, s)
	qp := _pt_sub(b0, a0)
	if math.abs(den) <= EPS {
		if math.abs(_cross(qp, r)) <= EPS do return .collinear, 0, 0
		return .none, 0, 0
	}
	ta = _cross(qp, s) / den
	tb = _cross(qp, r) / den
	if ta >= -EPS && ta <= 1.0 + EPS && tb >= -EPS && tb <= 1.0 + EPS {
		return .intersect, math.clamp(ta, 0.0, 1.0), math.clamp(tb, 0.0, 1.0)
	}
	return .none, ta, tb
}

@(private = "file")
_line_project_t :: proc "contextless" (a0, a1, p: [2]f64) -> f64 {
	d := _pt_sub(a1, a0)
	den := _dot(d, d)
	if den <= EPS do return 0.0
	return _dot(_pt_sub(p, a0), d) / den
}

@(private = "file")
_append_intersections :: proc(
	a: Segment,
	b: Segment,
	params_a, params_b: ^ParamList,
) -> Clipper_Error {
	if a.kind == .Line && b.kind == .Line {
		kind, ta, tb := _line_intersection_params(a.pts[0], a.pts[1], b.pts[0], b.pts[1])
		if kind == .intersect {
			_append_param(params_a, ta) or_return
			_append_param(params_b, tb) or_return
		} else if kind == .collinear {
			b_points := [2][2]f64{b.pts[0], b.pts[1]}
			for p in b_points {
				t := _line_project_t(a.pts[0], a.pts[1], p)
				if t >= -EPS && t <= 1.0 + EPS do _append_param(params_a, t) or_return
			}
			a_points := [2][2]f64{a.pts[0], a.pts[1]}
			for p in a_points {
				t := _line_project_t(b.pts[0], b.pts[1], p)
				if t >= -EPS && t <= 1.0 + EPS do _append_param(params_b, t) or_return
			}
		}
		return nil
	}

	count, _, ta, tb := linalg_ex.GetBezierIntersectPt(a.kind, a.pts, b.kind, b.pts)
	for i in 0 ..< count {
		_append_param(params_a, ta[i]) or_return
		_append_param(params_b, tb[i]) or_return
	}
	return nil
}

@(private = "file")
_filled_from_wind :: #force_inline proc "contextless" (wind: int, fill_rule: FillRule) -> bool {
	switch fill_rule {
	case .EvenOdd:
		return (wind & 1) != 0
	case .NonZero:
		return wind != 0
	case .Positive:
		return wind > 0
	case .Negative:
		return wind < 0
	}
	return false
}

@(private = "file")
_winding_at :: proc(point: [2]f64, segs: []Segment, path_type: PathType) -> int {
	query_y := point.y + SIDE_EPS * 0.37
	max_x := point.x + 1.0
	for s in segs {
		if s.is_open || s.path_type != path_type do continue
		for i in 0 ..= _end_index(s.kind) {
			if s.pts[i].x > max_x do max_x = s.pts[i].x
		}
	}
	wind := 0
	ray := [4][2]f64{{point.x, query_y}, {max_x + 1.0, query_y}, {}, {}}
	for s in segs {
		if s.is_open || s.path_type != path_type do continue
		if s.kind == .Line {
			y0 := s.pts[0].y
			y1 := s.pts[1].y
			if math.abs(y1 - y0) <= EPS do continue
			crosses := (y0 <= query_y && y1 > query_y) || (y1 <= query_y && y0 > query_y)
			if !crosses do continue
			x := s.pts[0].x + (query_y - y0) * (s.pts[1].x - s.pts[0].x) / (y1 - y0)
			if x > point.x + EPS do wind += y1 > y0 ? 1 : -1
			continue
		}

		count, _, t_ray, t_curve := linalg_ex.GetBezierIntersectPt(.Line, ray, s.kind, s.pts)
		for i in 0 ..< count {
			if t_ray[i] <= EPS || t_ray[i] > 1.0 + EPS do continue
			t_seg := t_curve[i]
			if t_seg <= 1e-12 || t_seg >= 1.0 - 1e-12 do continue
			ip := _eval(s.kind, s.pts, t_seg)
			if ip.x <= point.x + EPS do continue
			tan := _tangent(s.kind, s.pts, t_seg)
			if math.abs(tan.y) <= EPS do continue
			wind += tan.y > 0 ? 1 : -1
		}
	}
	return wind
}

@(private = "file")
_filled_at :: proc(point: [2]f64, segs: []Segment, path_type: PathType, fill_rule: FillRule) -> bool {
	return _filled_from_wind(_winding_at(point, segs, path_type), fill_rule)
}

@(private = "file")
_clip_result :: #force_inline proc "contextless" (clip_type: ClipType, subj, clip: bool) -> bool {
	switch clip_type {
	case .Union:
		return subj || clip
	case .Intersection:
		return subj && clip
	case .Difference:
		return subj && !clip
	case .Xor:
		return subj != clip
	}
	return false
}

@(private = "file")
_open_contributes :: #force_inline proc "contextless" (clip_type: ClipType, in_subj, in_clip: bool) -> bool {
	switch clip_type {
	case .Intersection:
		return in_clip
	case .Union:
		return !in_subj && !in_clip
	case .Difference, .Xor:
		return !in_clip
	}
	return false
}

@(private = "file")
_append_piece_points :: proc(
	path: ^[dynamic][2]f64,
	curves: ^[dynamic]bool,
	piece: Piece,
	first: bool,
) -> Clipper_Error {
	if first {
		non_zero_append(path, piece.pts[0]) or_return
		if curves != nil do non_zero_append(curves, false) or_return
	}
	switch piece.kind {
	case .Line:
	case .Quad:
		non_zero_append(path, piece.pts[1]) or_return
		if curves != nil do non_zero_append(curves, true) or_return
	case .Cubic:
		non_zero_append(path, piece.pts[1], piece.pts[2]) or_return
		if curves != nil do non_zero_append(curves, true, true) or_return
	}
	non_zero_append(path, _seg_end_piece(piece)) or_return
	if curves != nil do non_zero_append(curves, false) or_return
	return nil
}

@(private = "file")
_trim_duplicate_closure :: proc(path: ^[dynamic][2]f64, curves: ^[dynamic]bool) -> Clipper_Error {
	if len(path^) > 1 && _pt_eq(path^[0], path^[len(path^) - 1]) {
		non_zero_resize_dynamic_array(path, len(path^) - 1) or_return
		if curves != nil do non_zero_resize_dynamic_array(curves, len(curves^) - 1) or_return
	}
	return nil
}

@(private = "file")
_build_closed_paths :: proc(
	pieces: ^[dynamic]Piece,
	has_curves: bool,
) -> (
	paths: [dynamic][dynamic][2]f64,
	curves_out: [dynamic][dynamic]bool,
	err: Clipper_Error,
) {
	paths = make([dynamic][dynamic][2]f64, context.temp_allocator) or_return
	if has_curves do curves_out = make([dynamic][dynamic]bool, context.temp_allocator) or_return

	for i in 0 ..< len(pieces) {
		if pieces[i].used do continue

		path := make([dynamic][2]f64, context.temp_allocator) or_return
		curves: [dynamic]bool
		if has_curves do curves = make([dynamic]bool, context.temp_allocator) or_return

		first_pt := _seg_start(pieces[i])
		current := _seg_end_piece(pieces[i])
		pieces[i].used = true
		_append_piece_points(&path, has_curves ? &curves : nil, pieces[i], true) or_return

		for !_pt_eq(current, first_pt) {
			next_idx := -1
			for j in 0 ..< len(pieces) {
				if pieces[j].used do continue
				if _pt_eq(_seg_start(pieces[j]), current) {
					next_idx = j
					break
				}
			}
			if next_idx < 0 do break
			pieces[next_idx].used = true
			_append_piece_points(&path, has_curves ? &curves : nil, pieces[next_idx], false) or_return
			current = _seg_end_piece(pieces[next_idx])
		}

		_trim_duplicate_closure(&path, has_curves ? &curves : nil) or_return
		if len(path) >= 3 {
			non_zero_append(&paths, path) or_return
			if has_curves do non_zero_append(&curves_out, curves) or_return
		}
	}
	return
}

@(private = "file")
_build_open_paths :: proc(
	pieces: []Piece,
	has_curves: bool,
) -> (
	paths: [dynamic][dynamic][2]f64,
	curves_out: [dynamic][dynamic]bool,
	err: Clipper_Error,
) {
	paths = make([dynamic][dynamic][2]f64, context.temp_allocator) or_return
	if has_curves do curves_out = make([dynamic][dynamic]bool, context.temp_allocator) or_return

	path := make([dynamic][2]f64, context.temp_allocator) or_return
	curves: [dynamic]bool
	if has_curves do curves = make([dynamic]bool, context.temp_allocator) or_return
	have_path := false
	current: [2]f64

	for piece in pieces {
		if !have_path || !_pt_eq(current, _seg_start(piece)) {
			if have_path && len(path) >= 2 {
				non_zero_append(&paths, path) or_return
				if has_curves do non_zero_append(&curves_out, curves) or_return
			}
			path = make([dynamic][2]f64, context.temp_allocator) or_return
			if has_curves do curves = make([dynamic]bool, context.temp_allocator) or_return
			_append_piece_points(&path, has_curves ? &curves : nil, piece, true) or_return
			have_path = true
		} else {
			_append_piece_points(&path, has_curves ? &curves : nil, piece, false) or_return
		}
		current = _seg_end_piece(piece)
	}
	if have_path && len(path) >= 2 {
		non_zero_append(&paths, path) or_return
		if has_curves do non_zero_append(&curves_out, curves) or_return
	}
	return
}

@(private = "file")
_split_all_segments :: proc(
	segs: []Segment,
) -> (
	pieces: [dynamic]Piece,
	err: Clipper_Error,
) {
	params := make([dynamic]ParamList, context.temp_allocator) or_return
	non_zero_resize(&params, len(segs)) or_return
	for i in 0 ..< len(segs) {
		params[i] = make(ParamList, context.temp_allocator) or_return
		_append_param(&params[i], 0.0) or_return
		_append_param(&params[i], 1.0) or_return
	}

	for i in 0 ..< len(segs) {
		for j in i + 1 ..< len(segs) {
			if segs[i].is_open && segs[j].is_open do continue
			_append_intersections(segs[i], segs[j], &params[i], &params[j]) or_return
		}
	}

	pieces = make([dynamic]Piece, context.temp_allocator) or_return
	for s, i in segs {
		_sort_params(&params[i])
		for j in 0 ..< len(params[i]) - 1 {
			t0 := params[i][j]
			t1 := params[i][j + 1]
			if t1 - t0 <= EPS do continue
			p := Piece {
				pts       = _extract_sub(s.kind, s.pts, t0, t1),
				kind      = s.kind,
				path_type = s.path_type,
				is_open   = s.is_open,
				path_idx  = s.path_idx,
				order     = s.order,
			}
			non_zero_append(&pieces, p) or_return
		}
	}
	return
}

@(private = "file")
_classify_pieces :: proc(
	pieces: []Piece,
	closed_segs: []Segment,
	clip_type: ClipType,
	fill_rule: FillRule,
) -> (
	closed_pieces: [dynamic]Piece,
	open_pieces: [dynamic]Piece,
	err: Clipper_Error,
) {
	closed_pieces = make([dynamic]Piece, context.temp_allocator) or_return
	open_pieces = make([dynamic]Piece, context.temp_allocator) or_return

	for piece in pieces {
		mid := _eval(piece.kind, piece.pts, 0.5)
		tan := _tangent(piece.kind, piece.pts, 0.5)
		len_sq := _dot(tan, tan)
		if len_sq <= EPS do continue

		if piece.is_open {
			in_subj := _filled_at(mid, closed_segs, .Subject, fill_rule)
			in_clip := _filled_at(mid, closed_segs, .Clip, fill_rule)
			if _open_contributes(clip_type, in_subj, in_clip) {
				non_zero_append(&open_pieces, piece) or_return
			}
			continue
		}

		scale := SIDE_EPS / math.sqrt(len_sq)
		normal := [2]f64{-tan.y * scale, tan.x * scale}
		left := _pt_add(mid, normal)
		right := _pt_sub(mid, normal)

		left_subj := _filled_at(left, closed_segs, .Subject, fill_rule)
		left_clip := _filled_at(left, closed_segs, .Clip, fill_rule)
		right_subj := _filled_at(right, closed_segs, .Subject, fill_rule)
		right_clip := _filled_at(right, closed_segs, .Clip, fill_rule)

		left_res := _clip_result(clip_type, left_subj, left_clip)
		right_res := _clip_result(clip_type, right_subj, right_clip)
		if left_res == right_res do continue

		out_piece := piece
		if !left_res && right_res do out_piece = _reverse_piece(out_piece)
		non_zero_append(&closed_pieces, out_piece) or_return
	}
	return
}

@(private = "file")
_copy_solution_paths :: proc(
	has_curves: bool,
	paths: [dynamic][dynamic][2]f64,
	curves: [dynamic][dynamic]bool,
	allocator := context.allocator,
) -> (
	out_paths: [][][2]f64,
	out_curves: [][]bool,
	err: Clipper_Error,
) {
	if len(paths) == 0 do return nil, nil, nil
	out_paths = utils_private.make_non_zeroed_slice([][][2]f64, len(paths), allocator) or_return
	if has_curves do out_curves = utils_private.make_non_zeroed_slice([][]bool, len(paths), allocator) or_return
	for path, i in paths {
		out_paths[i] = utils_private.make_non_zeroed_slice([][2]f64, len(path), allocator) or_return
		for p, j in path do out_paths[i][j] = p
		if has_curves {
			out_curves[i] = utils_private.make_non_zeroed_slice([]bool, len(curves[i]), allocator) or_return
			for v, j in curves[i] do out_curves[i][j] = v
		}
	}
	return
}

@(private = "file")
_boolean_curve64 :: proc(
	clip_type: ClipType,
	subjects: [][][2]f64,
	clips: [][][2]f64,
	opens: [][][2]f64,
	subjects_is_curves: [][]bool,
	clips_is_curves: [][]bool,
	opens_is_curves: [][]bool,
	fill_rule: FillRule,
	allocator := context.allocator,
) -> (
	res: [][][2]f64,
	res_open: [][][2]f64,
	res_is_curves: [][]bool,
	res_open_is_curves: [][]bool,
	err: Clipper_Error,
) {
	segs := make([dynamic]Segment, context.temp_allocator) or_return
	order := 0
	if subjects != nil do _append_path_segments(&segs, subjects, subjects_is_curves, .Subject, false, &order) or_return
	if clips != nil do _append_path_segments(&segs, clips, clips_is_curves, .Clip, false, &order) or_return
	if opens != nil do _append_path_segments(&segs, opens, opens_is_curves, .Subject, true, &order) or_return
	if len(segs) == 0 do return nil, nil, nil, nil, nil

	closed_segs := make([dynamic]Segment, context.temp_allocator) or_return
	for s in segs {
		if !s.is_open do non_zero_append(&closed_segs, s) or_return
	}

	all_pieces := _split_all_segments(segs[:]) or_return
	closed_pieces, open_pieces := _classify_pieces(all_pieces[:], closed_segs[:], clip_type, fill_rule) or_return

	has_closed_curves := subjects_is_curves != nil || clips_is_curves != nil
	has_open_curves := opens_is_curves != nil
	solution_closed, solution_closed_curves := _build_closed_paths(&closed_pieces, has_closed_curves) or_return
	solution_open, solution_open_curves := _build_open_paths(open_pieces[:], has_open_curves) or_return

	res, res_is_curves = _copy_solution_paths(has_closed_curves, solution_closed, solution_closed_curves, allocator) or_return
	res_open, res_open_is_curves = _copy_solution_paths(has_open_curves, solution_open, solution_open_curves, allocator) or_return
	return
}

BooleanOpCurve :: proc(
	clip_type: ClipType,
	subjects: [][][2]$T,
	clips: [][][2]T,
	opens: [][][2]T,
	subjects_is_curves: [][]bool,
	clips_is_curves: [][]bool,
	opens_is_curves: [][]bool,
	fill_rule: FillRule = .NonZero,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	res_open: [][][2]T,
	res_is_curves: [][]bool,
	res_open_is_curves: [][]bool,
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	sub64 := _to_f64_paths(subjects) or_return
	clip64 := _to_f64_paths(clips) or_return
	open64 := _to_f64_paths(opens) or_return

	res64, open_res64, curve64, open_curve64 := _boolean_curve64(
		clip_type,
		sub64,
		clip64,
		open64,
		subjects_is_curves,
		clips_is_curves,
		opens_is_curves,
		fill_rule,
		context.temp_allocator,
	) or_return

	res = _from_f64_paths(T, res64, allocator) or_return
	res_open = _from_f64_paths(T, open_res64, allocator) or_return
	res_is_curves = _copy_bool_paths(curve64, allocator) or_return
	res_open_is_curves = _copy_bool_paths(open_curve64, allocator) or_return
	return
}

BooleanOp :: proc(
	clip_type: ClipType,
	subjects: [][][2]$T,
	clips: [][][2]T,
	opens: [][][2]T,
	fill_rule: FillRule = .NonZero,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	res_open: [][][2]T,
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	res, res_open, _, _ = BooleanOpCurve(
		clip_type,
		subjects,
		clips,
		opens,
		nil,
		nil,
		nil,
		fill_rule,
		allocator,
	) or_return
	return
}

RectClip :: proc(
	rect: linalg_ex.Rect_($T),
	paths: [][][2]T,
	path_is_curves: [][]bool = nil,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	res_is_curves: [][]bool,
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	rect_path := [4][2]T {
		{rect.left, rect.top},
		{rect.right, rect.top},
		{rect.right, rect.bottom},
		{rect.left, rect.bottom},
	}
	res, _, res_is_curves, _, err = BooleanOpCurve(
		.Intersection,
		paths,
		[][][2]T{rect_path[:]},
		nil,
		path_is_curves,
		nil,
		nil,
		.NonZero,
		allocator,
	)
	return
}

RectClipLines :: proc(
	rect: linalg_ex.Rect_($T),
	lines: [][][2]T,
	line_is_curves: [][]bool = nil,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	res_is_curves: [][]bool,
	err: Clipper_Error,
) where intrinsics.type_is_float(T) {
	rect_path := [4][2]T {
		{rect.left, rect.top},
		{rect.right, rect.top},
		{rect.right, rect.bottom},
		{rect.left, rect.bottom},
	}
	_, res, _, res_is_curves, err = BooleanOpCurve(
		.Intersection,
		nil,
		[][][2]T{rect_path[:]},
		lines,
		nil,
		nil,
		line_is_curves,
		.NonZero,
		allocator,
	)
	return
}
