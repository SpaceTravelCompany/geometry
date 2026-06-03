package clipper

import "../linalg_ex"
import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:slice"
import "shared:utils_private"

// ──── Error ──────────────────────────────────────────────────────────────────

__ClipperError :: enum {
	FAILED,
	TOO_SMALL,
	LENGTH_MISMATCH,
}

ClipperError :: union #shared_nil {
	__ClipperError,
	runtime.Allocator_Error,
}

// ──── Public Enums ───────────────────────────────────────────────────────────

FillRule :: enum u8 {
	EvenOdd,
	NonZero,
	Positive,
	Negative,
}

ClipType :: enum u8 {
	NoClip,
	Intersection,
	Union,
	Difference,
	Xor,
}

JoinType :: enum u8 {
	Square,
	Bevel,
	Round,
	Miter,
}

EndType :: enum u8 {
	Polygon,
	Joined,
	Butt,
	Square,
	Round,
}

// ──── Private Enums ──────────────────────────────────────────────────────────

@(private = "file")
PathType :: enum u8 {
	Subject,
	Clip,
}

@(private = "file")
_VertexFlags :: enum u32 {
	Empty     = 0,
	OpenStart = 1,
	OpenEnd   = 2,
	LocalMax  = 4,
	LocalMin  = 8,
}

@(private = "file")
_JoinWith :: enum u8 {
	NoJoin,
	Left,
	Right,
}

@(private = "file")
_Location :: enum u8 {
	Left   = 0,
	Top    = 1,
	Right  = 2,
	Bottom = 3,
	Inside = 4,
}

@(private = "file")
_PointInPolygonResult :: enum u8 {
	IsOn,
	IsInside,
	IsOutside,
}

// ──── Constants ──────────────────────────────────────────────────────────────

@(private = "file")
EPS :: 1e-9

@(private = "file")
SideEps :: 1e-7

@(private = "file")
ArcConst :: 0.002

// ──── Z‑type Helper ──────────────────────────────────────────────────────────
// Z_None is used when the user's point type has no z‑component.
// Z_Active (f64) is used when the user's point type has z.

@(private = "file")
Z_None :: struct {}

@(private = "file")
Z_Active :: f64

// ──── Internal Generic Point ─────────────────────────────────────────────────
// _Point($Z) is the single internal point representation.
//   Z = struct{}  →  zero‑size z, no z overhead
//   Z = f64       →  full z support
//
// All geometry math uses only x and y regardless of Z.

@(private = "file")
_Point :: struct($Z: typeid) {
	x, y: f64,
	z:    Z,
}

// ──── Internal Types (all generic over $Z) ───────────────────────────────────

@(private = "file")
_Vertex :: struct($Z: typeid) {
	pt:    _Point(Z),
	next:  ^_Vertex(Z),
	prev:  ^_Vertex(Z),
	flags: _VertexFlags,
}

@(private = "file")
_LocalMinima :: struct($Z: typeid) {
	vertex:   ^_Vertex(Z),
	polytype: PathType,
	is_open:  bool,
}

@(private = "file")
_OutPt :: struct($Z: typeid) {
	pt:     _Point(Z),
	next:   ^_OutPt(Z),
	prev:   ^_OutPt(Z),
	outrec: ^_OutRec(Z),
	horz:   ^_HorzSegment(Z),
}

@(private = "file")
_OutRec :: struct($Z: typeid) {
	idx:        int,
	owner:      ^_OutRec(Z),
	front_edge: ^_Active(Z),
	back_edge:  ^_Active(Z),
	pts:        ^_OutPt(Z),
	bounds:     _RectF64,
	path:       [dynamic]_Point(Z),
	is_open:    bool,
}

@(private = "file")
_Active :: struct($Z: typeid) {
	bot:           _Point(Z),
	top:           _Point(Z),
	curr_x:        f64,
	dx:            f64,
	wind_dx:       i32,
	wind_cnt:      i32,
	wind_cnt2:     i32,
	outrec:        ^_OutRec(Z),
	prev_in_ael:   ^_Active(Z),
	next_in_ael:   ^_Active(Z),
	prev_in_sel:   ^_Active(Z),
	next_in_sel:   ^_Active(Z),
	jump:          ^_Active(Z),
	vertex_top:    ^_Vertex(Z),
	local_min:     ^_LocalMinima(Z),
	is_left_bound: bool,
	join_with:     _JoinWith,
}

@(private = "file")
_IntersectNode :: struct($Z: typeid) {
	pt:    _Point(Z),
	edge1: ^_Active(Z),
	edge2: ^_Active(Z),
}

@(private = "file")
_HorzSegment :: struct($Z: typeid) {
	left_op:       ^_OutPt(Z),
	right_op:      ^_OutPt(Z),
	left_to_right: bool,
}

@(private = "file")
_HorzJoin :: struct($Z: typeid) {
	op1: ^_OutPt(Z),
	op2: ^_OutPt(Z),
}

// ──── Internal RectF64 (always f64, no Z needed) ─────────────────────────────

@(private = "file")
_RectF64 :: struct {
	left, top, right, bottom: f64,
}

// ──── Group for Offset Engine ────────────────────────────────────────────────

@(private = "file")
_Group :: struct($Z: typeid) {
	paths_in:        [dynamic][dynamic]_Point(Z),
	lowest_path_idx: int, // -1 means none
	is_reversed:     bool,
	join_type:       JoinType,
	end_type:        EndType,
}

// ──── ClipperOffset (offset engine) ──────────────────────────────────────────

@(private = "file")
_ClipperOffset :: struct($Z: typeid) {
	delta_:              f64,
	group_delta_:        f64,
	temp_lim_:           f64,
	steps_per_rad_:      f64,
	step_sin_:           f64,
	step_cos_:           f64,
	norms:               [dynamic]_Point(Z_None), // unit normals: always no z
	path_out:            [dynamic]_Point(Z),
	solution:            ^[dynamic][]_Point(Z),
	groups_:             [dynamic]_Group(Z),
	join_type_:          JoinType,
	end_type_:           EndType,
	miter_limit_:        f64,
	arc_tolerance_:      f64,
	preserve_collinear_: bool,
	reverse_solution_:   bool,
	error_code_:         int,
	zCallback_:          rawptr, // user's z callback (type-erased, nil when no Z)
}

// ──── ClipperBase (boolean engine) ───────────────────────────────────────────

@(private = "file")
_ClipperBase :: struct($Z: typeid) {
	cliptype_:            ClipType,
	fillrule_:            FillRule,
	fillpos:              FillRule,
	bot_y_:               f64,
	minima_list_sorted_:  bool,
	using_polytree_:      bool,
	actives_:             ^_Active(Z),
	sel_:                 ^_Active(Z),
	minima_list_:         [dynamic]^_LocalMinima(Z),
	current_locmin_iter_: int,
	vertex_lists_:        [dynamic]^_Vertex(Z),
	scanline_list_:       [dynamic]f64,
	intersect_nodes_:     [dynamic]_IntersectNode(Z),
	horz_seg_list_:       [dynamic]_HorzSegment(Z),
	horz_join_list_:      [dynamic]_HorzJoin(Z),
	outrec_list_:         [dynamic]^_OutRec(Z),
	preserve_collinear_:  bool,
	reverse_solution_:    bool,
	error_code_:          int,
	has_open_paths_:      bool,
	succeeded_:           bool,

	defaultZ: f64,
	zCallback_: rawptr, // user's z callback (type-erased, nil when no Z)
}

// ──── OutPt2 + RectClip (RectClip uses its own light-weight types) ────────────

@(private = "file")
_OutPt2 :: struct {
	pt:        _Point(Z_None), // RectClip always works in 2D
	owner_idx: int,
	edge:      ^[dynamic]^_OutPt2,
	next:      ^_OutPt2,
	prev:      ^_OutPt2,
}

@(private = "file")
_RectClip64 :: struct {
	rect_:         _RectF64,
	rect_as_path_: [dynamic]_Point(Z_None),
	rect_mp_:      _Point(Z_None),
	path_bounds_:  _RectF64,
	op_container_: [dynamic]_OutPt2,
	results_:      [dynamic]^_OutPt2,
	edges_:        [8][dynamic]^_OutPt2,
	start_locs_:   [dynamic]_Location,
}

@(private = "file")
_RectClipLines64 :: struct {
	using _: _RectClip64,
}

// ──── Generic Point Accessors (work on any user PointT) ──────────────────────

@(private = "file")
_pointHasZ :: proc($PointT: typeid) -> bool {
	return(
		len(PointT) >=
		3 when intrinsics.type_is_array(PointT) else intrinsics.type_has_field(
			PointT,
			"z",
		) when intrinsics.type_is_struct(PointT) else false \
	)
}

@(private = "file")
_pointX :: #force_inline proc "contextless" (p: $PointT) -> f64 {
	return f64(p.x)
}

@(private = "file")
_pointY :: #force_inline proc "contextless" (p: $PointT) -> f64 {
	return f64(p.y)
}

@(private = "file")
_pointZ :: #force_inline proc "contextless" (p: $PointT) -> f64 {
	HasZ ::
		len(PointT) >=
		3 when intrinsics.type_is_array(PointT) else intrinsics.type_has_field(
			PointT,
			"z",
		) when intrinsics.type_is_struct(PointT) else false
	when HasZ {
		return f64(p.z)
	}
	return 0
}

@(private = "file")
_makePoint :: #force_inline proc "contextless" ($PointT: typeid, x, y: f64, z: f64 = 0) -> PointT {
	HasZ ::
		len(PointT) >=
		3 when intrinsics.type_is_array(PointT) else intrinsics.type_has_field(
			PointT,
			"z",
		) when intrinsics.type_is_struct(PointT) else false
	p: PointT
	p.x = type_of(p.x)(x)
	p.y = type_of(p.y)(y)
	when HasZ {
		p.z = type_of(p.z)(z)
	}
	return p
}

// Convert user → internal: helpers for Phase 4
// (implemented when BooleanOp is written)

// ──── Point2 Math Helpers (work on _Point($Z), only x,y used) ────────────────

@(private = "file")
_xy :: #force_inline proc "contextless" (p: $P) -> _Point(Z_None) {
	return {x = f64(p.x), y = f64(p.y)}
}

@(private = "file")
_samePoint :: #force_inline proc "contextless" (a, b: $P) -> bool {
	dx := f64(a.x) - f64(b.x)
	dy := f64(a.y) - f64(b.y)
	return dx * dx + dy * dy <= EPS * EPS
}

@(private = "file")
_ptAdd :: #force_inline proc "contextless" (a, b: _Point(Z_None)) -> _Point(Z_None) {
	return {x = a.x + b.x, y = a.y + b.y}
}

@(private = "file")
_ptSub :: #force_inline proc "contextless" (a, b: _Point(Z_None)) -> _Point(Z_None) {
	return {x = a.x - b.x, y = a.y - b.y}
}

@(private = "file")
_ptMul :: #force_inline proc "contextless" (a: _Point(Z_None), s: f64) -> _Point(Z_None) {
	return {x = a.x * s, y = a.y * s}
}

@(private = "file")
_cross :: #force_inline proc "contextless" (a, b: _Point(Z_None)) -> f64 {
	return a.x * b.y - a.y * b.x
}

@(private = "file")
_dot :: #force_inline proc "contextless" (a, b: _Point(Z_None)) -> f64 {
	return a.x * b.x + a.y * b.y
}

@(private = "file")
_lenSq :: #force_inline proc "contextless" (a: _Point(Z_None)) -> f64 {
	return a.x * a.x + a.y * a.y
}

@(private = "file")
_crossProduct :: #force_inline proc "contextless" (a, b, c: _Point(Z_None)) -> f64 {
	return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)
}

@(private = "file")
_dotProduct :: #force_inline proc "contextless" (a, b, c: _Point(Z_None)) -> f64 {
	return (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y)
}

@(private = "file")
_distanceSqr :: #force_inline proc "contextless" (a, b: _Point(Z_None)) -> f64 {
	dx := a.x - b.x
	dy := a.y - b.y
	return dx * dx + dy * dy
}

@(private = "file")
_areaPath :: proc "contextless" (path: []$PointT) -> f64 {
	if len(path) < 3 do return 0
	area: f64
	for i in 0 ..< len(path) {
		j := (i + 1) % len(path)
		area += _pointX(path[i]) * _pointY(path[j]) - _pointX(path[j]) * _pointY(path[i])
	}
	return area * 0.5
}

@(private = "file")
_reversePath :: proc(path: []$PointT) {
	for i := 0; i < len(path) / 2; i += 1 {
		j := len(path) - 1 - i
		path[i], path[j] = path[j], path[i]
	}
}

@(private = "file")
_areaOutPt :: proc(op: ^_OutPt($Z)) -> f64 {
	if op == nil {return 0}
	result: f64
	op2 := op
	for {
		result += (op2.prev.pt.y + op2.pt.y) * (op2.prev.pt.x - op2.pt.x)
		op2 = op2.next
		if op2 == op {break}
	}
	return result * 0.5
}

@(private = "file")
_areaTriangle :: #force_inline proc "contextless" (a, b, c: _Point(Z_None)) -> f64 {
	return (c.y + a.y) * (c.x - a.x) + (a.y + b.y) * (a.x - b.x) + (b.y + c.y) * (b.x - c.x)
}

// ──── Geometry Predicates ────────────────────────────────────────────────────
// All work on _Point(Z_None) (i.e. only x,y) since geometry is 2D only.

@(private = "file")
_getDx :: #force_inline proc "contextless" (pt1, pt2: _Point(Z_None)) -> f64 {
	dy := pt2.y - pt1.y
	if abs(dy) > EPS {
		return (pt2.x - pt1.x) / dy
	} else if pt2.x > pt1.x {
		return -math.F64_MAX
	} else {
		return math.F64_MAX
	}
}

@(private = "file")
_topX :: #force_inline proc "contextless" (
	dx: f64,
	bot, top: _Point(Z_None),
	currentY: f64,
) -> f64 {
	if currentY == top.y || top.x == bot.x {
		return top.x
	} else if currentY == bot.y {
		return bot.x
	} else {
		return bot.x + dx * (currentY - bot.y)
	}
}

@(private = "file")
_isHorizontalE :: #force_inline proc "contextless" (e: ^_Active($Z)) -> bool {
	return abs(e.top.y - e.bot.y) < EPS
}

@(private = "file")
_isHorizontalPt :: #force_inline proc "contextless" (p1, p2: _Point(Z_None)) -> bool {
	return abs(p1.y - p2.y) < EPS
}

@(private = "file")
_isHeadingRightHorz :: #force_inline proc "contextless" (e: ^_Active($Z)) -> bool {
	return e.dx == -math.F64_MAX
}

@(private = "file")
_isHeadingLeftHorz :: #force_inline proc "contextless" (e: ^_Active($Z)) -> bool {
	return e.dx == math.F64_MAX
}

@(private = "file")
_setDx :: #force_inline proc "contextless" (e: ^_Active($Z)) {
	e.dx = _getDx(_xy(e.bot), _xy(e.top))
}

@(private = "file")
_nextVertex :: #force_inline proc "contextless" (e: ^_Active($Z)) -> ^_Vertex(Z) {
	if e.wind_dx > 0 {
		return e.vertex_top.next
	} else {
		return e.vertex_top.prev
	}
}

@(private = "file")
_prevPrevVertex :: #force_inline proc "contextless" (ae: ^_Active($Z)) -> ^_Vertex(Z) {
	if ae.wind_dx > 0 {
		return ae.vertex_top.prev.prev
	} else {
		return ae.vertex_top.next.next
	}
}

@(private = "file")
_isCollinear :: #force_inline proc "contextless" (a, sharedPt, b: _Point(Z_None)) -> bool {
	return abs(_crossProduct(a, sharedPt, b)) < EPS
}

@(private = "file")
_crossProductSign :: #force_inline proc "contextless" (a, b, c: _Point(Z_None)) -> int {
	cp := _crossProduct(a, b, c)
	if cp > EPS {return 1}
	if cp < -EPS {return -1}
	return 0
}

@(private = "file")
_getLineIntersectPt :: proc(ln1a, ln1b, ln2a, ln2b: _Point(Z_None), ip: ^_Point(Z_None)) -> bool {
	dx1 := ln1b.x - ln1a.x
	dy1 := ln1b.y - ln1a.y
	dx2 := ln2b.x - ln2a.x
	dy2 := ln2b.y - ln2a.y

	det := dy1 * dx2 - dy2 * dx1
	if abs(det) < EPS {return false}

	t := ((ln1a.x - ln2a.x) * dy2 - (ln1a.y - ln2a.y) * dx2) / det
	if t <= 0.0 {
		ip^ = ln1a
	} else if t >= 1.0 {
		ip^ = ln1b
	} else {
		ip.x = ln1a.x + t * dx1
		ip.y = ln1a.y + t * dy1
	}
	return true
}

@(private = "file")
_segmentsIntersect :: proc(seg1a, seg1b, seg2a, seg2b: _Point(Z_None), inclusive: bool) -> bool {
	dy1 := seg1b.y - seg1a.y
	dx1 := seg1b.x - seg1a.x
	dy2 := seg2b.y - seg2a.y
	dx2 := seg2b.x - seg2a.x
	cp := dy1 * dx2 - dy2 * dx1
	if abs(cp) < EPS {return false}

	t := (seg1a.x - seg2a.x) * dy2 - (seg1a.y - seg2a.y) * dx2
	if inclusive {
		if abs(t) < EPS {return true}
		if t > 0 {
			if cp < 0 || t > cp {return false}
		} else if cp > 0 || t < cp {return false}
	} else {
		if abs(t) < EPS {return false}
		if t > 0 {
			if cp < 0 || t >= cp {return false}
		} else if cp > 0 || t <= cp {return false}
	}

	t = (seg1a.x - seg2a.x) * dy1 - (seg1a.y - seg2a.y) * dx1
	if inclusive {
		if abs(t) < EPS {return true}
		if t > 0 {return cp > 0 && t <= cp} else {return cp < 0 && t >= cp}
	} else {
		if abs(t) < EPS {return false}
		if t > 0 {return cp > 0 && t < cp} else {return cp < 0 && t > cp}
	}
}

@(private = "file")
_perpendicDistFromLineSqrd :: proc(pt, line1, line2: _Point(Z_None)) -> f64 {
	a := pt.x - line1.x
	b := pt.y - line1.y
	c := line2.x - line1.x
	d := line2.y - line1.y
	if c == 0 && d == 0 {return 0}
	return (a * d - c * b) * (a * d - c * b) / (c * c + d * d)
}

@(private = "file")
_translatePoint :: #force_inline proc "contextless" (
	pt: _Point(Z_None),
	dx, dy: f64,
) -> _Point(Z_None) {
	return {x = pt.x + dx, y = pt.y + dy}
}

@(private = "file")
_reflectPoint :: #force_inline proc "contextless" (pt, pivot: _Point(Z_None)) -> _Point(Z_None) {
	return {x = pivot.x + (pivot.x - pt.x), y = pivot.y + (pivot.y - pt.y)}
}

@(private = "file")
_getClosestPointOnSegment :: proc(offPt, seg1, seg2: _Point(Z_None)) -> _Point(Z_None) {
	if seg1.x == seg2.x && seg1.y == seg2.y {return seg1}
	dx := seg2.x - seg1.x
	dy := seg2.y - seg1.y
	q := ((offPt.x - seg1.x) * dx + (offPt.y - seg1.y) * dy) / (dx * dx + dy * dy)
	if q < 0 {q = 0}
	if q > 1 {q = 1}
	return {x = seg1.x + q * dx, y = seg1.y + q * dy}
}

@(private = "file")
_pointsReallyClose :: #force_inline proc "contextless" (a, b: _Point(Z_None)) -> bool {
	return abs(a.x - b.x) < SideEps && abs(a.y - b.y) < SideEps
}

// ──── Vertex flag helpers ────────────────────────────────────────────────────

@(private = "file")
_hasFlag :: #force_inline proc "contextless" (v: ^_Vertex($Z), f: _VertexFlags) -> bool {
	return (u32(v.flags) & u32(f)) != 0
}

@(private = "file")
_setFlag :: #force_inline proc "contextless" (v: ^_Vertex($Z), f: _VertexFlags) {
	v.flags = _VertexFlags(u32(v.flags) | u32(f))
}

// ──── Active edge helpers ────────────────────────────────────────────────────

@(private = "file")
_getPolyType :: #force_inline proc "contextless" (e: ^_Active($Z)) -> PathType {
	return e.local_min.polytype
}

@(private = "file")
_isSamePolyType :: #force_inline proc "contextless" (e1, e2: ^_Active($Z)) -> bool {
	return e1.local_min.polytype == e2.local_min.polytype
}

@(private = "file")
_isOdd :: #force_inline proc "contextless" (val: int) -> bool {
	return (val & 1) != 0
}

@(private = "file")
_isHotEdge :: #force_inline proc "contextless" (e: ^_Active($Z)) -> bool {
	return e.outrec != nil
}

@(private = "file")
_isOpen :: #force_inline proc "contextless" (e: ^_Active($Z)) -> bool {
	return e.local_min.is_open
}

@(private = "file")
_isOpenEnd :: #force_inline proc "contextless" (v: ^_Vertex($Z)) -> bool {
	return _hasFlag(v, .OpenStart) || _hasFlag(v, .OpenEnd)
}

@(private = "file")
_isOpenEndE :: #force_inline proc "contextless" (e: ^_Active($Z)) -> bool {
	return _isOpenEnd(e.vertex_top)
}

@(private = "file")
_getPrevHotEdge :: proc(e: ^_Active($Z)) -> ^_Active(Z) {
	prev := e.prev_in_ael
	for prev != nil && (_isOpen(prev) || !_isHotEdge(prev)) {
		prev = prev.prev_in_ael
	}
	return prev
}

@(private = "file")
_isFront :: #force_inline proc "contextless" (e: ^_Active($Z)) -> bool {
	return e == e.outrec.front_edge
}

@(private = "file")
_isInvalidPath :: #force_inline proc "contextless" (op: ^_OutPt($Z)) -> bool {
	return op == nil || op.next == op
}

@(private = "file")
_isMaximaV :: #force_inline proc "contextless" (v: ^_Vertex($Z)) -> bool {
	return _hasFlag(v, .LocalMax)
}

@(private = "file")
_isMaximaE :: #force_inline proc "contextless" (e: ^_Active($Z)) -> bool {
	return _isMaximaV(e.vertex_top)
}

@(private = "file")
_isJoined :: #force_inline proc "contextless" (e: ^_Active($Z)) -> bool {
	return e.join_with != .NoJoin
}

@(private = "file")
_pointCount :: proc(op: ^_OutPt($Z)) -> int {
	if op == nil {return 0}
	op2 := op
	cnt := 0
	for {
		op2 = op2.next
		cnt += 1
		if op2 == op {break}
	}
	return cnt
}

@(private = "file")
_isValidClosedPath :: proc(op: ^_OutPt($Z)) -> bool {
	if op == nil {return false}
	if op.next == op {return false}
	if op.next == op.prev {return false}
	return !_isVerySmallTriangle(op)
}

@(private = "file")
_isVerySmallTriangle :: proc(op: ^_OutPt($Z)) -> bool {
	if op.next.next != op.prev {return false}
	return(
		_pointsReallyClose(_xy(op.prev.pt), _xy(op.next.pt)) ||
		_pointsReallyClose(_xy(op.pt), _xy(op.next.pt)) ||
		_pointsReallyClose(_xy(op.pt), _xy(op.prev.pt)) \
	)
}

// ──── ClipperBase Engine Methods ─────────────────────────────────────────────

// _addLocMin: append a local minima vertex (idempotent)
@(private = "file")
_addLocMin :: proc(
	locMinList: ^[dynamic]^_LocalMinima($Z),
	vert: ^_Vertex(Z),
	polytype: PathType,
	is_open: bool,
) {
	if _hasFlag(vert, .LocalMin) { return }
	_setFlag(vert, .LocalMin)
	lm := new(_LocalMinima(Z), context.temp_allocator)
	if lm == nil { return }
	lm.vertex = vert
	lm.polytype = polytype
	lm.is_open = is_open
	append(locMinList, lm)
}

// _addPaths_: convert input paths into internal Vertex linked-list + local minima
@(private = "file")
_addPaths_ :: proc(
	paths: [dynamic][dynamic]_Point($Z),
	polytype: PathType,
	is_open: bool,
	vertexLists: ^[dynamic]^_Vertex(Z),
	locMinList: ^[dynamic]^_LocalMinima(Z),
) {
	// count total vertices
	total_vertex_count := 0
	for path in paths {
		total_vertex_count += len(path)
	}
	if total_vertex_count == 0 { return }

	// allocate one contiguous block for all vertices (temp allocator)
	allVertices := make([]_Vertex(Z), total_vertex_count, context.temp_allocator)
	if allVertices == nil { return }
	vi := 0 // global vertex index into allVertices

	for path_idx in 0 ..< len(paths) {
		path := paths[path_idx]
		if len(path) == 0 { continue }

		// v0: first vertex index of this path (used to close the circle)
		v0_idx := vi
		v0 := &allVertices[v0_idx]

		curr_v := v0
		prev_v: ^_Vertex(Z) = nil
		curr_v.prev = nil
		cnt := 0

		for pt_idx in 0 ..< len(path) {
			pt := path[pt_idx]
			if prev_v != nil {
				// skip duplicate consecutive points (f64: EPS-based)
				if _samePoint(_xy(prev_v.pt), _xy(pt)) { continue }
				prev_v.next = curr_v
			}
			curr_v.prev = prev_v
			curr_v.pt = pt
			curr_v.flags = _VertexFlags.Empty
			prev_v = curr_v
			vi += 1
			cnt += 1
			if vi < total_vertex_count {
				curr_v = &allVertices[vi]
			}
		}

		// skip degenerate paths
		if prev_v == nil || prev_v.prev == nil { continue }

		// for closed paths: remove duplicated end vertex
		if !is_open && _samePoint(_xy(prev_v.pt), _xy(v0.pt)) {
			prev_v = prev_v.prev
		}

		// close the circular doubly-linked list
		prev_v.next = v0
		v0.prev = prev_v

		if cnt < 2 || (cnt == 2 && !is_open) { continue }

		// ── find and assign local minima ──
		going_up, going_up0: bool

		if is_open {
			curr_v = v0.next
			for curr_v != v0 && abs(curr_v.pt.y - v0.pt.y) < EPS {
				curr_v = curr_v.next
			}
			going_up = curr_v.pt.y <= v0.pt.y
			if going_up {
				_setFlag(v0, .OpenStart)
				_addLocMin(locMinList, v0, polytype, true)
			} else {
				_setFlag(v0, .OpenStart)
				_setFlag(v0, .LocalMax)
			}
		} else {
			// closed path: determine initial direction
			prev_v = v0.prev
			for prev_v != v0 && abs(prev_v.pt.y - v0.pt.y) < EPS {
				prev_v = prev_v.prev
			}
			if prev_v == v0 { continue } // completely flat path → skip
			going_up = prev_v.pt.y > v0.pt.y
		}

		going_up0 = going_up
		prev_v = v0
		curr_v = v0.next
		for curr_v != v0 {
			if curr_v.pt.y > prev_v.pt.y && going_up {
				_setFlag(prev_v, .LocalMax)
				going_up = false
			} else if curr_v.pt.y < prev_v.pt.y && !going_up {
				going_up = true
				_addLocMin(locMinList, prev_v, polytype, is_open)
			}
			prev_v = curr_v
			curr_v = curr_v.next
		}

		if is_open {
			if going_up {
				_setFlag(prev_v, .OpenEnd)
				_setFlag(prev_v, .LocalMax)
			} else {
				_setFlag(prev_v, .OpenEnd)
				_addLocMin(locMinList, prev_v, polytype, is_open)
			}
		} else if going_up != going_up0 {
			if going_up0 {
				_addLocMin(locMinList, prev_v, polytype, false)
			} else {
				_setFlag(prev_v, .LocalMax)
			}
		}
	} // end per-path loop

	append(vertexLists, &allVertices[0])
	return
}

// ──── ClipperBase lifecycle + scanline ops ───────────────────────────────────
// All internal allocations use context.temp_allocator — no manual free needed.
// Only output paths use context.allocator.

@(private = "file")
_clipperBase_init :: proc(cb: ^_ClipperBase($Z)) {
	cb^ = {}
	cb.minima_list_ = make([dynamic]^_LocalMinima(Z), context.temp_allocator)
	cb.vertex_lists_ = make([dynamic]^_Vertex(Z), context.temp_allocator)
	cb.scanline_list_ = make([dynamic]f64, context.temp_allocator)
	cb.intersect_nodes_ = make([dynamic]_IntersectNode(Z), context.temp_allocator)
	cb.horz_seg_list_ = make([dynamic]_HorzSegment(Z), context.temp_allocator)
	cb.horz_join_list_ = make([dynamic]_HorzJoin(Z), context.temp_allocator)
	cb.outrec_list_ = make([dynamic]^_OutRec(Z), context.temp_allocator)
	cb.preserve_collinear_ = true
	cb.succeeded_ = true
}

@(private = "file")
_clipperBase_destroy :: proc(cb: ^_ClipperBase($Z)) {
	cb^ = {}
}

// ──── scanline ops ───────────────────────────────────────────────────────────

@(private = "file")
_insertScanline :: proc(cb: ^_ClipperBase($Z), y: f64) {
	append(&cb.scanline_list_, y)
}

// pop the highest y from the scanline list (pops all duplicates at once)
@(private = "file")
_popScanline :: proc(cb: ^_ClipperBase($Z), y: ^f64) -> bool {
	sl := cb.scanline_list_
	if len(sl) == 0 { return false }
	max_y := sl[0]
	for i in 1 ..< len(sl) {
		if sl[i] > max_y { max_y = sl[i] }
	}
	// filter out all entries near max_y (removes duplicates)
	j := 0
	for i in 0 ..< len(sl) {
		if abs(sl[i] - max_y) > EPS {
			sl[j] = sl[i]
			j += 1
		}
	}
	cb.scanline_list_ = sl[:j]
	y^ = max_y
	return true
}

// pop local minima at the given y
@(private = "file")
_popLocalMinima :: proc(cb: ^_ClipperBase($Z), y: f64, local_minima: ^^_LocalMinima(Z)) -> bool {
	if cb.current_locmin_iter_ >= len(cb.minima_list_) { return false }
	lm := cb.minima_list_[cb.current_locmin_iter_]
	if abs(lm.vertex.pt.y - y) > EPS { return false }
	local_minima^ = lm
	cb.current_locmin_iter_ += 1
	return true
}

@(private = "file")
_reset :: proc(cb: ^_ClipperBase($Z)) {
	if !cb.minima_list_sorted_ {
		// sort by y descending, x ascending (LocMinSorter)
		slice.sort_by(cb.minima_list_[:], proc(a, b: ^_LocalMinima(Z)) -> bool {
			if abs(b.vertex.pt.y - a.vertex.pt.y) > EPS {
				return b.vertex.pt.y < a.vertex.pt.y // descending y
			}
			return b.vertex.pt.x > a.vertex.pt.x // descending x (for same y)
		})
		cb.minima_list_sorted_ = true
	}
	// push all minima y values as scanlines (in reverse)
	for i := len(cb.minima_list_) - 1; i >= 0; i -= 1 {
		_insertScanline(cb, cb.minima_list_[i].vertex.pt.y)
	}
	cb.current_locmin_iter_ = 0
	cb.actives_ = nil
	cb.sel_ = nil
	cb.succeeded_ = true
}

// ──── AEL operations ─────────────────────────────────────────────────────────

@(private = "file")
_isValidAelOrder :: proc(resident, newcomer: ^_Active($Z)) -> bool {
	if abs(newcomer.curr_x - resident.curr_x) > EPS {
		return newcomer.curr_x > resident.curr_x
	}
	// same curr_x → compare turning direction: resident.top → newcomer.bot → newcomer.top
	i := _crossProductSign(_xy(resident.top), _xy(newcomer.bot), _xy(newcomer.top))
	if i != 0 { return i < 0 }

	// collinear edges — check next turning direction
	if !_isMaximaE(resident) && resident.top.y > newcomer.top.y {
		nv := _nextVertex(resident)
		return _crossProductSign(_xy(newcomer.bot), _xy(resident.top), _xy(nv.pt)) <= 0
	} else if !_isMaximaE(newcomer) && newcomer.top.y > resident.top.y {
		nv := _nextVertex(newcomer)
		return _crossProductSign(_xy(newcomer.bot), _xy(newcomer.top), _xy(nv.pt)) >= 0
	}

	y := newcomer.bot.y
	newcomerIsLeft := newcomer.is_left_bound

	if abs(resident.bot.y - y) > EPS || abs(resident.local_min.vertex.pt.y - y) > EPS {
		return newcomer.is_left_bound
	} else if resident.is_left_bound != newcomerIsLeft {
		return newcomerIsLeft
	} else if _isCollinear(_xy(_prevPrevVertex(resident).pt), _xy(resident.bot), _xy(resident.top)) {
		return true
	} else {
		return (_crossProductSign(
			_xy(_prevPrevVertex(resident).pt),
			_xy(newcomer.bot),
			_xy(_prevPrevVertex(newcomer).pt)) > 0) == newcomerIsLeft
	}
}

@(private = "file")
_insertLeftEdge :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) {
	if cb.actives_ == nil {
		e.prev_in_ael = nil
		e.next_in_ael = nil
		cb.actives_ = e
	} else if !_isValidAelOrder(cb.actives_, e) {
		e.prev_in_ael = nil
		e.next_in_ael = cb.actives_
		cb.actives_.prev_in_ael = e
		cb.actives_ = e
	} else {
		e2 := cb.actives_
		for e2.next_in_ael != nil && _isValidAelOrder(e2.next_in_ael, e) {
			e2 = e2.next_in_ael
		}
		if e2.join_with == .Right {
			e2 = e2.next_in_ael
		}
		if e2 == nil { return }
		e.next_in_ael = e2.next_in_ael
		if e2.next_in_ael != nil { e2.next_in_ael.prev_in_ael = e }
		e.prev_in_ael = e2
		e2.next_in_ael = e
	}
}

@(private = "file")
_insertRightEdge :: proc(e, e2: ^_Active($Z)) {
	e2.next_in_ael = e.next_in_ael
	if e.next_in_ael != nil { e.next_in_ael.prev_in_ael = e2 }
	e2.prev_in_ael = e
	e.next_in_ael = e2
}

// remove edge from AEL and deallocate (temp allocator — just unlink, no free)
@(private = "file")
_deleteFromAEL :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) {
	prev := e.prev_in_ael
	nxt := e.next_in_ael
	if prev == nil && nxt == nil && e != cb.actives_ { return } // already deleted
	if prev != nil {
		prev.next_in_ael = nxt
	} else {
		cb.actives_ = nxt
	}
	if nxt != nil { nxt.prev_in_ael = prev }
	e.prev_in_ael = nil
	e.next_in_ael = nil
}

// precondition: e1 is immediately to the left of e2
@(private = "file")
_swapPositionsInAEL :: proc(cb: ^_ClipperBase($Z), e1, e2: ^_Active(Z)) {
	nxt := e2.next_in_ael
	if nxt != nil { nxt.prev_in_ael = e1 }
	prev := e1.prev_in_ael
	if prev != nil { prev.next_in_ael = e2 }
	e2.prev_in_ael = prev
	e2.next_in_ael = e1
	e1.prev_in_ael = e2
	e1.next_in_ael = nxt
	if e2.prev_in_ael == nil { cb.actives_ = e2 }
}

@(private = "file")
_pushHorz :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) {
	e.next_in_sel = cb.sel_
	cb.sel_ = e
}

@(private = "file")
_popHorz :: proc(cb: ^_ClipperBase($Z), e: ^^_Active(Z)) -> bool {
	if cb.sel_ == nil { return false }
	e^ = cb.sel_
	cb.sel_ = cb.sel_.next_in_sel
	return true
}

// ──── Wind count + Contribution ──────────────────────────────────────────────

@(private = "file")
_isContributingClosed :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) -> bool {
	#partial switch cb.fillrule_ {
	case .EvenOdd:
		break
	case .NonZero:
		if abs(e.wind_cnt) != 1 { return false }
	case .Positive:
		if e.wind_cnt != 1 { return false }
	case .Negative:
		if e.wind_cnt != -1 { return false }
	}

	#partial switch cb.cliptype_ {
	case .NoClip:
		return false
	case .Intersection:
		#partial switch cb.fillrule_ {
		case .Positive: return e.wind_cnt2 > 0
		case .Negative: return e.wind_cnt2 < 0
		case: return e.wind_cnt2 != 0
		}
	case .Union:
		#partial switch cb.fillrule_ {
		case .Positive: return e.wind_cnt2 <= 0
		case .Negative: return e.wind_cnt2 >= 0
		case: return e.wind_cnt2 == 0
		}
	case .Difference:
		result := false
		#partial switch cb.fillrule_ {
		case .Positive: result = e.wind_cnt2 <= 0
		case .Negative: result = e.wind_cnt2 >= 0
		case: result = e.wind_cnt2 == 0
		}
		if _getPolyType(e) == .Subject { return result }
		else { return !result }
	case .Xor:
		return true
	}
	return false
}

@(private = "file")
_isContributingOpen :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) -> bool {
	is_in_clip, is_in_subj: bool
	#partial switch cb.fillrule_ {
	case .Positive:
		is_in_clip = e.wind_cnt2 > 0
		is_in_subj = e.wind_cnt > 0
	case .Negative:
		is_in_clip = e.wind_cnt2 < 0
		is_in_subj = e.wind_cnt < 0
	case:
		is_in_clip = e.wind_cnt2 != 0
		is_in_subj = e.wind_cnt != 0
	}

	#partial switch cb.cliptype_ {
	case .Intersection: return is_in_clip
	case .Union: return !is_in_subj && !is_in_clip
	case: return !is_in_clip
	}
}

@(private = "file")
_setWindCountForClosedPathEdge :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) {
	// find the nearest closed path edge of the same PolyType in AEL (heading left)
	pt := _getPolyType(e)
	e2 := e.prev_in_ael
	for e2 != nil && (_getPolyType(e2) != pt || _isOpen(e2)) {
		e2 = e2.prev_in_ael
	}

	if e2 == nil {
		e.wind_cnt = e.wind_dx
		e2 = cb.actives_
	} else if cb.fillrule_ == .EvenOdd {
		e.wind_cnt = e.wind_dx
		e.wind_cnt2 = e2.wind_cnt2
		e2 = e2.next_in_ael
	} else {
		// NonZero, positive, or negative
		if e2.wind_cnt * e2.wind_dx < 0 {
			// opposite directions → e is outside e2
			if abs(e2.wind_cnt) > 1 {
				if e2.wind_dx * e.wind_dx < 0 {
					e.wind_cnt = e2.wind_cnt
				} else {
					e.wind_cnt = e2.wind_cnt + e.wind_dx
				}
			} else {
				e.wind_cnt = e.wind_dx
			}
		} else {
			// e must be inside e2
			if e2.wind_dx * e.wind_dx < 0 {
				e.wind_cnt = e2.wind_cnt
			} else {
				e.wind_cnt = e2.wind_cnt + e.wind_dx
			}
		}
		e.wind_cnt2 = e2.wind_cnt2
		e2 = e2.next_in_ael
	}

	// update wind_cnt2
	#partial switch cb.fillrule_ {
	case .EvenOdd:
		for e2 != e {
			if _getPolyType(e2) != pt && !_isOpen(e2) {
				e.wind_cnt2 = (e.wind_cnt2 == 0 ? 1 : 0)
			}
			e2 = e2.next_in_ael
		}
	case:
		for e2 != e {
			if _getPolyType(e2) != pt && !_isOpen(e2) {
				e.wind_cnt2 += e2.wind_dx
			}
			e2 = e2.next_in_ael
		}
	}
}

@(private = "file")
_setWindCountForOpenPathEdge :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) {
	if cb.fillrule_ == .EvenOdd {
		cnt1, cnt2: int
		e2 := cb.actives_
		for e2 != e {
			if _getPolyType(e2) == .Clip {
				cnt2 += 1
			} else if !_isOpen(e2) {
				cnt1 += 1
			}
			e2 = e2.next_in_ael
		}
		e.wind_cnt = (_isOdd(cnt1) ? 1 : 0)
		e.wind_cnt2 = (_isOdd(cnt2) ? 1 : 0)
	} else {
		e2 := cb.actives_
		for e2 != e {
			if _getPolyType(e2) == .Clip {
				e.wind_cnt2 += e2.wind_dx
			} else if !_isOpen(e2) {
				e.wind_cnt += e2.wind_dx
			}
			e2 = e2.next_in_ael
		}
	}
}

// ──── Output path construction ───────────────────────────────────────────────

@(private = "file")
_newOutRec :: proc(cb: ^_ClipperBase($Z)) -> ^_OutRec(Z) {
	rec := new(_OutRec(Z), context.temp_allocator)
	if rec == nil { return nil }
	rec.idx = len(cb.outrec_list_)
	rec.path = make([dynamic]_Point(Z), context.temp_allocator)
	append(&cb.outrec_list_, rec)
	return rec
}

@(private = "file")
_setSides :: proc(outrec: ^_OutRec($Z), start_edge, end_edge: ^_Active(Z)) {
	outrec.front_edge = start_edge
	outrec.back_edge = end_edge
}

@(private = "file")
_outrecIsAscending :: #force_inline proc "contextless" (hotEdge: ^_Active($Z)) -> bool {
	return hotEdge == hotEdge.outrec.front_edge
}

@(private = "file")
_swapFrontBackSides :: proc(outrec: ^_OutRec($Z)) {
	tmp := outrec.front_edge
	outrec.front_edge = outrec.back_edge
	outrec.back_edge = tmp
	if outrec.pts != nil {
		outrec.pts = outrec.pts.next
	}
}

@(private = "file")
_uncoupleOutRec :: proc(ae: ^_Active($Z)) {
	outrec := ae.outrec
	if outrec == nil { return }
	if outrec.front_edge != nil { outrec.front_edge.outrec = nil }
	if outrec.back_edge != nil { outrec.back_edge.outrec = nil }
	outrec.front_edge = nil
	outrec.back_edge = nil
}

@(private = "file")
_addOutPt :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z), pt: _Point(Z)) -> ^_OutPt(Z) {
	outrec := e.outrec
	to_front := _isFront(e)
	op_front := outrec.pts
	if op_front == nil { return nil }
	op_back := op_front.next

	if to_front {
		if _samePoint(_xy(op_front.pt), _xy(pt)) { return op_front }
	} else if _samePoint(_xy(op_back.pt), _xy(pt)) {
		return op_back
	}

	new_op := new(_OutPt(Z), context.temp_allocator)
	if new_op == nil { return nil }
	new_op.pt = pt
	new_op.outrec = outrec

	op_back.prev = new_op
	new_op.prev = op_front
	new_op.next = op_back
	op_front.next = new_op
	if to_front { outrec.pts = new_op }
	return new_op
}

@(private = "file")
_addLocalMinPoly :: proc(cb: ^_ClipperBase($Z), e1, e2: ^_Active(Z), pt: _Point(Z), is_new: bool) -> ^_OutPt(Z) {
	outrec := _newOutRec(cb)
	if outrec == nil { return nil }
	e1.outrec = outrec
	e2.outrec = outrec

	if _isOpen(e1) {
		outrec.owner = nil
		outrec.is_open = true
		if e1.wind_dx > 0 {
			_setSides(outrec, e1, e2)
		} else {
			_setSides(outrec, e2, e1)
		}
	} else {
		prevHotEdge := _getPrevHotEdge(e1)
		if prevHotEdge != nil {
			if _outrecIsAscending(prevHotEdge) == is_new {
				_setSides(outrec, e2, e1)
			} else {
				_setSides(outrec, e1, e2)
			}
		} else {
			outrec.owner = nil
			if is_new {
				_setSides(outrec, e1, e2)
			} else {
				_setSides(outrec, e2, e1)
			}
		}
	}

	op := new(_OutPt(Z), context.temp_allocator)
	if op == nil { return nil }
	op.pt = pt
	op.outrec = outrec
	op.next = op
	op.prev = op
	outrec.pts = op
	return op
}

@(private = "file")
_addLocalMaxPoly :: proc(cb: ^_ClipperBase($Z), e1, e2: ^_Active(Z), pt: _Point(Z)) -> ^_OutPt(Z) {
	if _isJoined(e1) { _split(cb, e1, pt) }
	if _isJoined(e2) { _split(cb, e2, pt) }

	if _isFront(e1) == _isFront(e2) {
		if _isOpenEndE(e1) {
			_swapFrontBackSides(e1.outrec)
		} else if _isOpenEndE(e2) {
			_swapFrontBackSides(e2.outrec)
		} else {
			cb.succeeded_ = false
			return nil
		}
	}

	result := _addOutPt(cb, e1, pt)
	if e1.outrec == e2.outrec {
		outrec := e1.outrec
		outrec.pts = result
		_uncoupleOutRec(e1)
		result = outrec.pts
		if outrec.owner != nil && outrec.owner.front_edge == nil {
			outrec.owner = _getRealOutRec(outrec.owner)
		}
	} else if _isOpen(e1) {
		if e1.wind_dx < 0 {
			_joinOutrecPaths(cb, e1, e2)
		} else {
			_joinOutrecPaths(cb, e2, e1)
		}
	} else if e1.outrec.idx < e2.outrec.idx {
		_joinOutrecPaths(cb, e1, e2)
	} else {
		_joinOutrecPaths(cb, e2, e1)
	}
	return result
}

@(private = "file")
_joinOutrecPaths :: proc(cb: ^_ClipperBase($Z), e1, e2: ^_Active(Z)) {
	p1_st := e1.outrec.pts
	p2_st := e2.outrec.pts
	p1_end := p1_st.next
	p2_end := p2_st.next
	if _isFront(e1) {
		p2_end.prev = p1_st
		p1_st.next = p2_end
		p2_st.next = p1_end
		p1_end.prev = p2_st
		e1.outrec.pts = p2_st
		e1.outrec.front_edge = e2.outrec.front_edge
		if e1.outrec.front_edge != nil {
			e1.outrec.front_edge.outrec = e1.outrec
		}
	} else {
		p1_end.prev = p2_st
		p2_st.next = p1_end
		p1_st.next = p2_end
		p2_end.prev = p1_st
		e1.outrec.back_edge = e2.outrec.back_edge
		if e1.outrec.back_edge != nil {
			e1.outrec.back_edge.outrec = e1.outrec
		}
	}

	e2.outrec.front_edge = nil
	e2.outrec.back_edge = nil
	e2.outrec.pts = nil

	if _isOpenEndE(e1) {
		e2.outrec.pts = e1.outrec.pts
		e1.outrec.pts = nil
	} else {
		// SetOwner with cycle detection (port of C++ SetOwner)
		owner := e1.outrec
		for owner != nil && owner.pts == nil { owner = owner.owner }
		tmp := owner
		for tmp != nil && tmp != e2.outrec { tmp = tmp.owner }
		if tmp != nil { owner = e2.outrec.owner }
		e2.outrec.owner = owner
	}

	e1.outrec = nil
	e2.outrec = nil
}

@(private = "file")
_startOpenPath :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z), pt: _Point(Z)) -> ^_OutPt(Z) {
	outrec := _newOutRec(cb)
	if outrec == nil { return nil }
	outrec.is_open = true
	if e.wind_dx > 0 {
		outrec.front_edge = e
		outrec.back_edge = nil
	} else {
		outrec.front_edge = nil
		outrec.back_edge = e
	}
	e.outrec = outrec

	op := new(_OutPt(Z), context.temp_allocator)
	if op == nil { return nil }
	op.pt = pt
	op.outrec = outrec
	op.next = op
	op.prev = op
	outrec.pts = op
	return op
}

@(private = "file")
_getRealOutRec :: proc(outrec: ^_OutRec($Z)) -> ^_OutRec(Z) {
	for outrec != nil && outrec.pts == nil {
		outrec = outrec.owner
	}
	return outrec
}

@(private = "file")
_split :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z), pt: _Point(Z)) {
	if e.join_with == .Right {
		e.join_with = .NoJoin
		e.next_in_ael.join_with = .NoJoin
		_addLocalMinPoly(cb, e, e.next_in_ael, pt, true)
	} else {
		e.join_with = .NoJoin
		e.prev_in_ael.join_with = .NoJoin
		_addLocalMinPoly(cb, e.prev_in_ael, e, pt, true)
	}
}

// ──── Public API Stubs (to be implemented in later phases) ───────────────────

BooleanOp :: proc(
	clipType: ClipType,
	$PointT: typeid,
	subjects: [][]PointT,
	clips: [][]PointT,
	opens: [][]PointT,
	fillRule: FillRule = .NonZero,
	zCallback: proc(e1Bot, e1Top, e2Bot, e2Top: PointT, outPoint: ^PointT) = nil,
	allocator := context.allocator,
) -> (
	res: [][]PointT,
	resOpen: [][]PointT,
	err: ClipperError,
) {
	_ = clipType
	_ = subjects
	_ = clips
	_ = opens
	_ = fillRule
	_ = zCallback
	_ = allocator
	return nil, nil, __ClipperError.FAILED
}

RectClip :: proc(
	rect: $RectT,
	$PointT: typeid,
	closePaths: [][]PointT,
	openPaths: [][]PointT = nil,
	zCallback: proc(e1Bot, e1Top, e2Bot, e2Top: PointT, outPoint: ^PointT) = nil,
	allocator := context.allocator,
) -> (
	closed: [][]PointT,
	open: [][]PointT,
	err: ClipperError,
) {
	_ = rect
	_ = closePaths
	_ = openPaths
	_ = zCallback
	_ = allocator
	return nil, nil, __ClipperError.FAILED
}

InflatePaths :: proc(
	$PointT: typeid,
	closePaths: [][]PointT,
	openPaths: [][]PointT = nil,
	delta: f64,
	joinType: JoinType,
	endType: EndType = .Polygon,
	miterLimit: f64 = 2.0,
	arcTolerance: f64 = 0.0,
	preserveCollinear: bool = false,
	reverseSolution: bool = false,
	zCallback: proc(e1Bot, e1Top, e2Bot, e2Top: PointT, outPoint: ^PointT) = nil,
	allocator := context.allocator,
) -> (
	res: [][]PointT,
	err: ClipperError,
) {
	_ = closePaths
	_ = openPaths
	_ = delta
	_ = joinType
	_ = endType
	_ = miterLimit
	_ = arcTolerance
	_ = preserveCollinear
	_ = reverseSolution
	_ = zCallback
	_ = allocator
	return nil, __ClipperError.FAILED
}
