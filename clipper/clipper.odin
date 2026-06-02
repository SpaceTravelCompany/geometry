package clipper

import "../linalg_ex"
import "base:intrinsics"
import "base:runtime"
import "core:math"
import "shared:utils_private"

__ClipperError :: enum {
	FAILED,
	CANT_FIRST_CURVE,
	OPEN_CURVE_END,
	TOO_SMALL,
	LENGTH_MISMATCH,
}

ClipperError :: union #shared_nil {
	__ClipperError,
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
	pathType: PathType,
	isOpen:   bool,
	pathIdx:  int,
	segIdx:   int,
	order:     int,
}

@(private = "file")
Piece :: struct {
	pts:       [4][2]f64,
	kind:      linalg_ex.BezierKind,
	pathType: PathType,
	isOpen:   bool,
	pathIdx:  int,
	order:     int,
	used:      bool,
}

@(private = "file")
ParamList :: [dynamic]f64

@(private = "file")
EPS :: 1e-9

@(private = "file")
SideEps :: 1e-7

@(private = "file")
_endIndex :: #force_inline proc "contextless" (kind: linalg_ex.BezierKind) -> int {
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
_ptEq :: #force_inline proc "contextless" (a, b: [2]f64) -> bool {
	return math.abs(a.x - b.x) <= EPS && math.abs(a.y - b.y) <= EPS
}

@(private = "file")
_ptAdd :: #force_inline proc "contextless" (a, b: [2]f64) -> [2]f64 {
	return {a.x + b.x, a.y + b.y}
}

@(private = "file")
_ptSub :: #force_inline proc "contextless" (a, b: [2]f64) -> [2]f64 {
	return {a.x - b.x, a.y - b.y}
}

@(private = "file")
_ptMul :: #force_inline proc "contextless" (a: [2]f64, s: f64) -> [2]f64 {
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
_segStart :: #force_inline proc "contextless" (p: Piece) -> [2]f64 {
	return p.pts[0]
}

@(private = "file")
_segEndPiece :: #force_inline proc "contextless" (p: Piece) -> [2]f64 {
	return p.pts[_endIndex(p.kind)]
}

@(private = "file")
_segEnd :: #force_inline proc "contextless" (s: Segment) -> [2]f64 {
	return s.pts[_endIndex(s.kind)]
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
		return _ptSub(pts[1], pts[0])
	}
	return linalg_ex.EvalBezierTangent(kind, pts, t)
}

@(private = "file")
_extractSub :: proc "contextless" (
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
	tAdj := (t1 - t0) / denom
	if tAdj >= 1.0 - EPS do return result

	switch kind {
	case .Line:
		p := linalg_ex.SubdivLine([2][2]f64{result[0], result[1]}, tAdj)
		result[1] = p
	case .Quad:
		p01, m, _ := linalg_ex.SubdivQuadraticBezier([3][2]f64{result[0], result[1], result[2]}, tAdj)
		result = {result[0], p01, m, {}}
	case .Cubic:
		c0, c1, m, _, _ := linalg_ex.SubdivCubicBezier(result, tAdj)
		result = {result[0], c0, c1, m}
	}
	return result
}

@(private = "file")
_reversePiece :: proc "contextless" (p: Piece) -> Piece {
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
_appendParam :: proc(list: ^ParamList, t: f64) -> ClipperError {
	if t < -EPS || t > 1.0 + EPS do return nil
	tt := math.clamp(t, 0.0, 1.0)
	for v in list[:] {
		if math.abs(v - tt) <= EPS do return nil
	}
	non_zero_append(list, tt) or_return
	return nil
}

@(private = "file")
_sortParams :: proc(list: ^ParamList) {
	for i in 1 ..< len(list) {
		j := i
		for j > 0 && list[j - 1] > list[j] {
			list[j - 1], list[j] = list[j], list[j - 1]
			j -= 1
		}
	}
}

@(private = "file")
_appendXMonotoneParams :: proc(list: ^ParamList, kind: linalg_ex.BezierKind, pts: [4][2]f64) -> ClipperError {
	if kind == .Line do return nil
	t0, t1 := linalg_ex.GetBezierTForXMonotone(kind, [4]f64{pts[0].x, pts[1].x, pts[2].x, pts[3].x})
	_appendParam(list, t0) or_return
	_appendParam(list, t1) or_return
	return nil
}

@(private = "file")
_appendSegmentFromParams :: proc(
	out: ^[dynamic]Segment,
	seg: Segment,
	params: ParamList,
) -> ClipperError {
	paramsSorted := params
	_sortParams(&paramsSorted)
	for i in 0 ..< len(paramsSorted) - 1 {
		t0 := paramsSorted[i]
		t1 := paramsSorted[i + 1]
		if t1 - t0 <= EPS do continue
		s := seg
		s.pts = _extractSub(seg.kind, seg.pts, t0, t1)
		non_zero_append(out, s) or_return
	}
	return nil
}

@(private = "file")
_appendRawSegment :: proc(
	out: ^[dynamic]Segment,
	seg: Segment,
) -> ClipperError {
	params := make(ParamList, context.temp_allocator) or_return
	_appendParam(&params, 0.0) or_return
	_appendParam(&params, 1.0) or_return
	_appendXMonotoneParams(&params, seg.kind, seg.pts) or_return
	_appendSegmentFromParams(out, seg, params) or_return
	return nil
}

@(private = "file")
_appendPathSegments :: proc(
	out: ^[dynamic]Segment,
	paths: [][][2]f64,
	isCurves: [][]bool,
	pathType: PathType,
	isOpen: bool,
	order: ^int,
) -> ClipperError {
	for path, pathIdx in paths {
		if isOpen {
			if len(path) < 2 do return .TOO_SMALL
		} else {
			if len(path) < 3 do return .TOO_SMALL
		}

		curves: []bool
		if isCurves != nil {
			if pathIdx >= len(isCurves) || len(isCurves[pathIdx]) != len(path) do return .LENGTH_MISMATCH
			curves = isCurves[pathIdx]
			if len(curves) > 0 && curves[0] do return .CANT_FIRST_CURVE
		}

		j := 0
		segIdx := 0
		for j < len(path) {
			if isOpen && j >= len(path) - 1 do break

			seg := Segment {
				kind      = .Line,
				pathType = pathType,
				isOpen   = isOpen,
				pathIdx  = pathIdx,
				segIdx   = segIdx,
				order     = order^,
			}
			seg.pts[0] = path[j]

			next := j + 1
			if next >= len(path) {
				if isOpen do break
				next = 0
			}

			if curves != nil && curves[next] {
				seg.pts[1] = path[next]
				next2 := next + 1
				if next2 >= len(path) {
					if isOpen do return .OPEN_CURVE_END
					next2 = 0
				}
				if curves[next2] {
					seg.pts[2] = path[next2]
					next3 := next2 + 1
					if next3 >= len(path) {
						if isOpen do return .OPEN_CURVE_END
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

			if !isOpen && j >= len(path) do j = len(path)
			_appendRawSegment(out, seg) or_return
			order^ += 1
			segIdx += 1
		}
	}
	return nil
}

@(private = "file")
_toF64Paths :: proc(paths: [][][2]$T) -> (out: [][][2]f64, err: ClipperError) where intrinsics.type_is_float(T) {
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
_fromF64Paths :: proc(
	$T: typeid,
	paths: [][][2]f64,
	allocator := context.allocator,
) -> (
	out: [][][2]T,
	err: ClipperError,
) where intrinsics.type_is_float(T) {
	if paths == nil do return nil, nil
	out = utils_private.makeNonZeroedSlice([][][2]T, len(paths), allocator) or_return
	for path, i in paths {
		out[i] = utils_private.makeNonZeroedSlice([][2]T, len(path), allocator) or_return
		for p, j in path {
			out[i][j] = {T(p.x), T(p.y)}
		}
	}
	return
}

@(private = "file")
_copyBoolPaths :: proc(paths: [][]bool, allocator := context.allocator) -> (out: [][]bool, err: ClipperError) {
	if paths == nil do return nil, nil
	out = utils_private.makeNonZeroedSlice([][]bool, len(paths), allocator) or_return
	for path, i in paths {
		out[i] = utils_private.makeNonZeroedSlice([]bool, len(path), allocator) or_return
		for v, j in path do out[i][j] = v
	}
	return
}

@(private = "file")
_lineIntersectionParams :: proc "contextless" (
	a0, a1, b0, b1: [2]f64,
) -> (
	kind: linalg_ex.IntersectKind,
	ta, tb: f64,
) {
	r := _ptSub(a1, a0)
	s := _ptSub(b1, b0)
	den := _cross(r, s)
	qp := _ptSub(b0, a0)
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
_lineProjectT :: proc "contextless" (a0, a1, p: [2]f64) -> f64 {
	d := _ptSub(a1, a0)
	den := _dot(d, d)
	if den <= EPS do return 0.0
	return _dot(_ptSub(p, a0), d) / den
}

@(private = "file")
_appendIntersections :: proc(
	a: Segment,
	b: Segment,
	paramsA, paramsB: ^ParamList,
) -> ClipperError {
	if a.kind == .Line && b.kind == .Line {
		kind, ta, tb := _lineIntersectionParams(a.pts[0], a.pts[1], b.pts[0], b.pts[1])
		if kind == .intersect {
			_appendParam(paramsA, ta) or_return
			_appendParam(paramsB, tb) or_return
		} else if kind == .collinear {
			bPoints := [2][2]f64{b.pts[0], b.pts[1]}
			for p in bPoints {
				t := _lineProjectT(a.pts[0], a.pts[1], p)
				if t >= -EPS && t <= 1.0 + EPS do _appendParam(paramsA, t) or_return
			}
			aPoints := [2][2]f64{a.pts[0], a.pts[1]}
			for p in aPoints {
				t := _lineProjectT(b.pts[0], b.pts[1], p)
				if t >= -EPS && t <= 1.0 + EPS do _appendParam(paramsB, t) or_return
			}
		}
		return nil
	}

	count, _, ta, tb := linalg_ex.GetBezierIntersectPt(a.kind, a.pts, b.kind, b.pts)
	for i in 0 ..< count {
		_appendParam(paramsA, ta[i]) or_return
		_appendParam(paramsB, tb[i]) or_return
	}
	return nil
}

@(private = "file")
_filledFromWind :: #force_inline proc "contextless" (wind: int, fillRule: FillRule) -> bool {
	switch fillRule {
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
_windingAt :: proc(point: [2]f64, segs: []Segment, pathType: PathType) -> int {
	queryY := point.y + SideEps * 0.37
	maxX := point.x + 1.0
	for s in segs {
		if s.isOpen || s.pathType != pathType do continue
		for i in 0 ..= _endIndex(s.kind) {
			if s.pts[i].x > maxX do maxX = s.pts[i].x
		}
	}
	wind := 0
	ray := [4][2]f64{{point.x, queryY}, {maxX + 1.0, queryY}, {}, {}}
	for s in segs {
		if s.isOpen || s.pathType != pathType do continue
		if s.kind == .Line {
			y0 := s.pts[0].y
			y1 := s.pts[1].y
			if math.abs(y1 - y0) <= EPS do continue
			crosses := (y0 <= queryY && y1 > queryY) || (y1 <= queryY && y0 > queryY)
			if !crosses do continue
			x := s.pts[0].x + (queryY - y0) * (s.pts[1].x - s.pts[0].x) / (y1 - y0)
			if x > point.x + EPS do wind += y1 > y0 ? 1 : -1
			continue
		}

		count, _, tRay, tCurve := linalg_ex.GetBezierIntersectPt(.Line, ray, s.kind, s.pts)
		for i in 0 ..< count {
			if tRay[i] <= EPS || tRay[i] > 1.0 + EPS do continue
			tSeg := tCurve[i]
			if tSeg <= 1e-12 || tSeg >= 1.0 - 1e-12 do continue
			ip := _eval(s.kind, s.pts, tSeg)
			if ip.x <= point.x + EPS do continue
			tan := _tangent(s.kind, s.pts, tSeg)
			if math.abs(tan.y) <= EPS do continue
			wind += tan.y > 0 ? 1 : -1
		}
	}
	return wind
}

@(private = "file")
_filledAt :: proc(point: [2]f64, segs: []Segment, pathType: PathType, fillRule: FillRule) -> bool {
	return _filledFromWind(_windingAt(point, segs, pathType), fillRule)
}

@(private = "file")
_clipResult :: #force_inline proc "contextless" (clipType: ClipType, subj, clip: bool) -> bool {
	switch clipType {
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
_openContributes :: #force_inline proc "contextless" (clipType: ClipType, inSubj, inClip: bool) -> bool {
	switch clipType {
	case .Intersection:
		return inClip
	case .Union:
		return !inSubj && !inClip
	case .Difference, .Xor:
		return !inClip
	}
	return false
}

@(private = "file")
_appendPiecePoints :: proc(
	path: ^[dynamic][2]f64,
	curves: ^[dynamic]bool,
	piece: Piece,
	first: bool,
) -> ClipperError {
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
	non_zero_append(path, _segEndPiece(piece)) or_return
	if curves != nil do non_zero_append(curves, false) or_return
	return nil
}

@(private = "file")
_trimDuplicateClosure :: proc(path: ^[dynamic][2]f64, curves: ^[dynamic]bool) -> ClipperError {
	if len(path^) > 1 && _ptEq(path^[0], path^[len(path^) - 1]) {
		non_zero_resize_dynamic_array(path, len(path^) - 1) or_return
		if curves != nil do non_zero_resize_dynamic_array(curves, len(curves^) - 1) or_return
	}
	return nil
}

@(private = "file")
_buildClosedPaths :: proc(
	pieces: ^[dynamic]Piece,
	hasCurves: bool,
) -> (
	paths: [dynamic][dynamic][2]f64,
	curvesOut: [dynamic][dynamic]bool,
	err: ClipperError,
) {
	paths = make([dynamic][dynamic][2]f64, context.temp_allocator) or_return
	if hasCurves do curvesOut = make([dynamic][dynamic]bool, context.temp_allocator) or_return

	for i in 0 ..< len(pieces) {
		if pieces[i].used do continue

		path := make([dynamic][2]f64, context.temp_allocator) or_return
		curves: [dynamic]bool
		if hasCurves do curves = make([dynamic]bool, context.temp_allocator) or_return

		firstPt := _segStart(pieces[i])
		current := _segEndPiece(pieces[i])
		pieces[i].used = true
		_appendPiecePoints(&path, hasCurves ? &curves : nil, pieces[i], true) or_return

		for !_ptEq(current, firstPt) {
			nextIdx := -1
			for j in 0 ..< len(pieces) {
				if pieces[j].used do continue
				if _ptEq(_segStart(pieces[j]), current) {
					nextIdx = j
					break
				}
			}
			if nextIdx < 0 do break
			pieces[nextIdx].used = true
			_appendPiecePoints(&path, hasCurves ? &curves : nil, pieces[nextIdx], false) or_return
			current = _segEndPiece(pieces[nextIdx])
		}

		_trimDuplicateClosure(&path, hasCurves ? &curves : nil) or_return
		if len(path) >= 3 {
			non_zero_append(&paths, path) or_return
			if hasCurves do non_zero_append(&curvesOut, curves) or_return
		}
	}
	return
}

@(private = "file")
_buildOpenPaths :: proc(
	pieces: []Piece,
	hasCurves: bool,
) -> (
	paths: [dynamic][dynamic][2]f64,
	curvesOut: [dynamic][dynamic]bool,
	err: ClipperError,
) {
	paths = make([dynamic][dynamic][2]f64, context.temp_allocator) or_return
	if hasCurves do curvesOut = make([dynamic][dynamic]bool, context.temp_allocator) or_return

	path := make([dynamic][2]f64, context.temp_allocator) or_return
	curves: [dynamic]bool
	if hasCurves do curves = make([dynamic]bool, context.temp_allocator) or_return
	havePath := false
	current: [2]f64

	for piece in pieces {
		if !havePath || !_ptEq(current, _segStart(piece)) {
			if havePath && len(path) >= 2 {
				non_zero_append(&paths, path) or_return
				if hasCurves do non_zero_append(&curvesOut, curves) or_return
			}
			path = make([dynamic][2]f64, context.temp_allocator) or_return
			if hasCurves do curves = make([dynamic]bool, context.temp_allocator) or_return
			_appendPiecePoints(&path, hasCurves ? &curves : nil, piece, true) or_return
			havePath = true
		} else {
			_appendPiecePoints(&path, hasCurves ? &curves : nil, piece, false) or_return
		}
		current = _segEndPiece(piece)
	}
	if havePath && len(path) >= 2 {
		non_zero_append(&paths, path) or_return
		if hasCurves do non_zero_append(&curvesOut, curves) or_return
	}
	return
}

@(private = "file")
_splitAllSegments :: proc(
	segs: []Segment,
) -> (
	pieces: [dynamic]Piece,
	err: ClipperError,
) {
	params := make([dynamic]ParamList, context.temp_allocator) or_return
	non_zero_resize(&params, len(segs)) or_return
	for i in 0 ..< len(segs) {
		params[i] = make(ParamList, context.temp_allocator) or_return
		_appendParam(&params[i], 0.0) or_return
		_appendParam(&params[i], 1.0) or_return
	}

	for i in 0 ..< len(segs) {
		for j in i + 1 ..< len(segs) {
			if segs[i].isOpen && segs[j].isOpen do continue
			_appendIntersections(segs[i], segs[j], &params[i], &params[j]) or_return
		}
	}

	pieces = make([dynamic]Piece, context.temp_allocator) or_return
	for s, i in segs {
		_sortParams(&params[i])
		for j in 0 ..< len(params[i]) - 1 {
			t0 := params[i][j]
			t1 := params[i][j + 1]
			if t1 - t0 <= EPS do continue
			p := Piece {
				pts       = _extractSub(s.kind, s.pts, t0, t1),
				kind      = s.kind,
				pathType = s.pathType,
				isOpen   = s.isOpen,
				pathIdx  = s.pathIdx,
				order     = s.order,
			}
			non_zero_append(&pieces, p) or_return
		}
	}
	return
}

@(private = "file")
_classifyPieces :: proc(
	pieces: []Piece,
	closedSegs: []Segment,
	clipType: ClipType,
	fillRule: FillRule,
) -> (
	closedPieces: [dynamic]Piece,
	openPieces: [dynamic]Piece,
	err: ClipperError,
) {
	closedPieces = make([dynamic]Piece, context.temp_allocator) or_return
	openPieces = make([dynamic]Piece, context.temp_allocator) or_return

	for piece in pieces {
		mid := _eval(piece.kind, piece.pts, 0.5)
		tan := _tangent(piece.kind, piece.pts, 0.5)
		lenSq := _dot(tan, tan)
		if lenSq <= EPS do continue

		if piece.isOpen {
			inSubj := _filledAt(mid, closedSegs, .Subject, fillRule)
			inClip := _filledAt(mid, closedSegs, .Clip, fillRule)
			if _openContributes(clipType, inSubj, inClip) {
				non_zero_append(&openPieces, piece) or_return
			}
			continue
		}

		scale := SideEps / math.sqrt(lenSq)
		normal := [2]f64{-tan.y * scale, tan.x * scale}
		left := _ptAdd(mid, normal)
		right := _ptSub(mid, normal)

		leftSubj := _filledAt(left, closedSegs, .Subject, fillRule)
		leftClip := _filledAt(left, closedSegs, .Clip, fillRule)
		rightSubj := _filledAt(right, closedSegs, .Subject, fillRule)
		rightClip := _filledAt(right, closedSegs, .Clip, fillRule)

		leftRes := _clipResult(clipType, leftSubj, leftClip)
		rightRes := _clipResult(clipType, rightSubj, rightClip)
		if leftRes == rightRes do continue

		outPiece := piece
		if !leftRes && rightRes do outPiece = _reversePiece(outPiece)
		non_zero_append(&closedPieces, outPiece) or_return
	}
	return
}

@(private = "file")
_copySolutionPaths :: proc(
	hasCurves: bool,
	paths: [dynamic][dynamic][2]f64,
	curves: [dynamic][dynamic]bool,
	allocator := context.allocator,
) -> (
	outPaths: [][][2]f64,
	outCurves: [][]bool,
	err: ClipperError,
) {
	if len(paths) == 0 do return nil, nil, nil
	outPaths = utils_private.makeNonZeroedSlice([][][2]f64, len(paths), allocator) or_return
	if hasCurves do outCurves = utils_private.makeNonZeroedSlice([][]bool, len(paths), allocator) or_return
	for path, i in paths {
		outPaths[i] = utils_private.makeNonZeroedSlice([][2]f64, len(path), allocator) or_return
		for p, j in path do outPaths[i][j] = p
		if hasCurves {
			outCurves[i] = utils_private.makeNonZeroedSlice([]bool, len(curves[i]), allocator) or_return
			for v, j in curves[i] do outCurves[i][j] = v
		}
	}
	return
}

@(private = "file")
_booleanCurve64 :: proc(
	clipType: ClipType,
	subjects: [][][2]f64,
	clips: [][][2]f64,
	opens: [][][2]f64,
	subjectsIsCurves: [][]bool,
	clipsIsCurves: [][]bool,
	opensIsCurves: [][]bool,
	fillRule: FillRule,
	allocator := context.allocator,
) -> (
	res: [][][2]f64,
	resOpen: [][][2]f64,
	resIsCurves: [][]bool,
	resOpenIsCurves: [][]bool,
	err: ClipperError,
) {
	segs := make([dynamic]Segment, context.temp_allocator) or_return
	order := 0
	if subjects != nil do _appendPathSegments(&segs, subjects, subjectsIsCurves, .Subject, false, &order) or_return
	if clips != nil do _appendPathSegments(&segs, clips, clipsIsCurves, .Clip, false, &order) or_return
	if opens != nil do _appendPathSegments(&segs, opens, opensIsCurves, .Subject, true, &order) or_return
	if len(segs) == 0 do return nil, nil, nil, nil, nil

	closedSegs := make([dynamic]Segment, context.temp_allocator) or_return
	for s in segs {
		if !s.isOpen do non_zero_append(&closedSegs, s) or_return
	}

	allPieces := _splitAllSegments(segs[:]) or_return
	closedPieces, openPieces := _classifyPieces(allPieces[:], closedSegs[:], clipType, fillRule) or_return

	hasClosedCurves := subjectsIsCurves != nil || clipsIsCurves != nil
	hasOpenCurves := opensIsCurves != nil
	solutionClosed, solutionClosedCurves := _buildClosedPaths(&closedPieces, hasClosedCurves) or_return
	solutionOpen, solutionOpenCurves := _buildOpenPaths(openPieces[:], hasOpenCurves) or_return

	res, resIsCurves = _copySolutionPaths(hasClosedCurves, solutionClosed, solutionClosedCurves, allocator) or_return
	resOpen, resOpenIsCurves = _copySolutionPaths(hasOpenCurves, solutionOpen, solutionOpenCurves, allocator) or_return
	return
}

BooleanOpCurve :: proc(
	clipType: ClipType,
	subjects: [][][2]$T,
	clips: [][][2]T,
	opens: [][][2]T,
	subjectsIsCurves: [][]bool,
	clipsIsCurves: [][]bool,
	opensIsCurves: [][]bool,
	fillRule: FillRule = .NonZero,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	resOpen: [][][2]T,
	resIsCurves: [][]bool,
	resOpenIsCurves: [][]bool,
	err: ClipperError,
) where intrinsics.type_is_float(T) {
	sub64 := _toF64Paths(subjects) or_return
	clip64 := _toF64Paths(clips) or_return
	open64 := _toF64Paths(opens) or_return

	res64, openRes64, curve64, openCurve64 := _booleanCurve64(
		clipType,
		sub64,
		clip64,
		open64,
		subjectsIsCurves,
		clipsIsCurves,
		opensIsCurves,
		fillRule,
		context.temp_allocator,
	) or_return

	res = _fromF64Paths(T, res64, allocator) or_return
	resOpen = _fromF64Paths(T, openRes64, allocator) or_return
	resIsCurves = _copyBoolPaths(curve64, allocator) or_return
	resOpenIsCurves = _copyBoolPaths(openCurve64, allocator) or_return
	return
}

BooleanOp :: proc(
	clipType: ClipType,
	subjects: [][][2]$T,
	clips: [][][2]T,
	opens: [][][2]T,
	fillRule: FillRule = .NonZero,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	resOpen: [][][2]T,
	err: ClipperError,
) where intrinsics.type_is_float(T) {
	res, resOpen, _, _ = BooleanOpCurve(
		clipType,
		subjects,
		clips,
		opens,
		nil,
		nil,
		nil,
		fillRule,
		allocator,
	) or_return
	return
}

RectClip :: proc(
	rect: linalg_ex.Rect_($T),
	paths: [][][2]T,
	pathIsCurves: [][]bool = nil,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	resIsCurves: [][]bool,
	err: ClipperError,
) where intrinsics.type_is_float(T) {
	rectPath := [4][2]T {
		{rect.left, rect.top},
		{rect.right, rect.top},
		{rect.right, rect.bottom},
		{rect.left, rect.bottom},
	}
	res, _, resIsCurves, _, err = BooleanOpCurve(
		.Intersection,
		paths,
		[][][2]T{rectPath[:]},
		nil,
		pathIsCurves,
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
	lineIsCurves: [][]bool = nil,
	allocator := context.allocator,
) -> (
	res: [][][2]T,
	resIsCurves: [][]bool,
	err: ClipperError,
) where intrinsics.type_is_float(T) {
	rectPath := [4][2]T {
		{rect.left, rect.top},
		{rect.right, rect.top},
		{rect.right, rect.bottom},
		{rect.left, rect.bottom},
	}
	_, res, _, resIsCurves, err = BooleanOpCurve(
		.Intersection,
		nil,
		[][][2]T{rectPath[:]},
		lines,
		nil,
		nil,
		lineIsCurves,
		.NonZero,
		allocator,
	)
	return
}
