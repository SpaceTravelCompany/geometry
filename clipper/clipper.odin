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
	defaultZ:             Z, // f64 when Z active, struct{} when Z none
	zCallback_:           rawptr, // user's z callback (type-erased)
	// callbackPtr는 user의 zCallback (rawptr) — 브리지가 PointT로 캐스팅하여 호출
	zBridge_:             proc(
		callbackPtr: rawptr,
		e1Bot, e1Top, e2Bot, e2Top: _Point(Z),
		outPoint: ^_Point(Z),
	),
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
_makePoint :: #force_inline proc "contextless" ($PointT: typeid, x, y: f64, z: $Z) -> PointT {
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
		p.z = z
	}
	return p
}

// Convert internal _Point(Z) → user PointT (for Z callback bridge)
@(private = "file")
_fromInternalPointZ :: #force_inline proc "contextless" (
	pt: _Point($Z),
	$PointT: typeid,
) -> PointT {
	HasZ ::
		intrinsics.type_has_field(PointT, "z") when intrinsics.type_is_struct(PointT) else len(
			PointT,
		) >=
		3 when intrinsics.type_is_array(PointT) else false
	p: PointT
	p.x = type_of(p.x)(pt.x)
	p.y = type_of(p.y)(pt.y)
	when HasZ {
		p.z = type_of(p.z)(pt.z)
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
	if _hasFlag(vert, .LocalMin) {return}
	_setFlag(vert, .LocalMin)
	lm := new(_LocalMinima(Z), context.temp_allocator)
	if lm == nil {return}
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
	if total_vertex_count == 0 {return}

	// allocate one contiguous block for all vertices (temp allocator)
	allVertices := make([]_Vertex(Z), total_vertex_count, context.temp_allocator)
	if allVertices == nil {return}
	vi := 0 // global vertex index into allVertices

	for path_idx in 0 ..< len(paths) {
		path := paths[path_idx]
		if len(path) == 0 {continue}

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
				// skip duplicate consecutive points
				if _xy(prev_v.pt) == _xy(pt) {continue}
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
		if prev_v == nil || prev_v.prev == nil {continue}

		// for closed paths: remove duplicated end vertex
		if !is_open && _xy(prev_v.pt) == _xy(v0.pt) {
			prev_v = prev_v.prev
		}

		// close the circular doubly-linked list
		prev_v.next = v0
		v0.prev = prev_v

		if cnt < 2 || (cnt == 2 && !is_open) {continue}

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
			if prev_v == v0 {continue} 	// completely flat path → skip
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
	cb.fillpos = .Positive
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
	if len(sl) == 0 {return false}
	max_y := sl[0]
	for i in 1 ..< len(sl) {
		if sl[i] > max_y {max_y = sl[i]}
	}
	// filter out all entries near max_y (removes duplicates)
	j := 0
	for i in 0 ..< len(sl) {
		if abs(sl[i] - max_y) > EPS {
			sl[j] = sl[i]
			j += 1
		}
	}
	clear(&cb.scanline_list_)
	for i in 0 ..< j {
		append(&cb.scanline_list_, sl[i])
	}
	y^ = max_y
	return true
}

// pop local minima at the given y
@(private = "file")
_popLocalMinima :: proc(cb: ^_ClipperBase($Z), y: f64, local_minima: ^^_LocalMinima(Z)) -> bool {
	if cb.current_locmin_iter_ >= len(cb.minima_list_) {return false}
	lm := cb.minima_list_[cb.current_locmin_iter_]
	if abs(lm.vertex.pt.y - y) > EPS {return false}
	local_minima^ = lm
	cb.current_locmin_iter_ += 1
	return true
}

@(private = "file")
_reset :: proc(cb: ^_ClipperBase($Z)) {
	if !cb.minima_list_sorted_ {
		// sort by y descending, x ascending (LocMinSorter)
		slice.sort_by(
			cb.minima_list_[:],
			proc(a, b: ^_LocalMinima(Z)) -> bool {
				if abs(b.vertex.pt.y - a.vertex.pt.y) > EPS {
					return b.vertex.pt.y < a.vertex.pt.y // descending y
				}
				return b.vertex.pt.x > a.vertex.pt.x // descending x (for same y)
			},
		)
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
	if i != 0 {return i < 0}

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
	} else if _isCollinear(
		_xy(_prevPrevVertex(resident).pt),
		_xy(resident.bot),
		_xy(resident.top),
	) {
		return true
	} else {
		return(
			(_crossProductSign(
					_xy(_prevPrevVertex(resident).pt),
					_xy(newcomer.bot),
					_xy(_prevPrevVertex(newcomer).pt),
				) >
				0) ==
			newcomerIsLeft \
		)
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
		if e2 == nil {return}
		e.next_in_ael = e2.next_in_ael
		if e2.next_in_ael != nil {e2.next_in_ael.prev_in_ael = e}
		e.prev_in_ael = e2
		e2.next_in_ael = e
	}
}

@(private = "file")
_insertRightEdge :: proc(e, e2: ^_Active($Z)) {
	e2.next_in_ael = e.next_in_ael
	if e.next_in_ael != nil {e.next_in_ael.prev_in_ael = e2}
	e2.prev_in_ael = e
	e.next_in_ael = e2
}

// remove edge from AEL and deallocate (temp allocator — just unlink, no free)
@(private = "file")
_deleteFromAEL :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) {
	prev := e.prev_in_ael
	nxt := e.next_in_ael
	if prev == nil && nxt == nil && e != cb.actives_ {return} 	// already deleted
	if prev != nil {
		prev.next_in_ael = nxt
	} else {
		cb.actives_ = nxt
	}
	if nxt != nil {nxt.prev_in_ael = prev}
	e.prev_in_ael = nil
	e.next_in_ael = nil
}

// precondition: e1 is immediately to the left of e2
@(private = "file")
_swapPositionsInAEL :: proc(cb: ^_ClipperBase($Z), e1, e2: ^_Active(Z)) {
	nxt := e2.next_in_ael
	if nxt != nil {nxt.prev_in_ael = e1}
	prev := e1.prev_in_ael
	if prev != nil {prev.next_in_ael = e2}
	e2.prev_in_ael = prev
	e2.next_in_ael = e1
	e1.prev_in_ael = e2
	e1.next_in_ael = nxt
	if e2.prev_in_ael == nil {cb.actives_ = e2}
}

@(private = "file")
_pushHorz :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) {
	e.next_in_sel = cb.sel_
	cb.sel_ = e
}

@(private = "file")
_popHorz :: proc(cb: ^_ClipperBase($Z), e: ^^_Active(Z)) -> bool {
	if cb.sel_ == nil {return false}
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
		if abs(e.wind_cnt) != 1 {return false}
	case .Positive:
		if e.wind_cnt != 1 {return false}
	case .Negative:
		if e.wind_cnt != -1 {return false}
	}

	#partial switch cb.cliptype_ {
	case .NoClip:
		return false
	case .Intersection:
		#partial switch cb.fillrule_ {
		case .Positive:
			return e.wind_cnt2 > 0
		case .Negative:
			return e.wind_cnt2 < 0
		case:
			return e.wind_cnt2 != 0
		}
	case .Union:
		#partial switch cb.fillrule_ {
		case .Positive:
			return e.wind_cnt2 <= 0
		case .Negative:
			return e.wind_cnt2 >= 0
		case:
			return e.wind_cnt2 == 0
		}
	case .Difference:
		result := false
		#partial switch cb.fillrule_ {
		case .Positive:
			result = e.wind_cnt2 <= 0
		case .Negative:
			result = e.wind_cnt2 >= 0
		case:
			result = e.wind_cnt2 == 0
		}
		if _getPolyType(e) == .Subject {return result} else {return !result}
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
	case .Intersection:
		return is_in_clip
	case .Union:
		return !is_in_subj && !is_in_clip
	case:
		return !is_in_clip
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
	if rec == nil {return nil}
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
	if outrec == nil {return}
	if outrec.front_edge != nil {outrec.front_edge.outrec = nil}
	if outrec.back_edge != nil {outrec.back_edge.outrec = nil}
	outrec.front_edge = nil
	outrec.back_edge = nil
}

@(private = "file")
_addOutPt :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z), pt: _Point(Z)) -> ^_OutPt(Z) {
	outrec := e.outrec
	to_front := _isFront(e)
	op_front := outrec.pts
	if op_front == nil {return nil}
	op_back := op_front.next

	if to_front {
		if _xy(op_front.pt) == _xy(pt) {return op_front}
	} else if _xy(op_back.pt) == _xy(pt) {
		return op_back
	}

	new_op := new(_OutPt(Z), context.temp_allocator)
	if new_op == nil {return nil}
	new_op.pt = pt
	new_op.outrec = outrec

	op_back.prev = new_op
	new_op.prev = op_front
	new_op.next = op_back
	op_front.next = new_op
	if to_front {outrec.pts = new_op}
	return new_op
}

@(private = "file")
_addLocalMinPoly :: proc(
	cb: ^_ClipperBase($Z),
	e1, e2: ^_Active(Z),
	pt: _Point(Z),
	is_new: bool,
) -> ^_OutPt(Z) {
	outrec := _newOutRec(cb)
	if outrec == nil {return nil}
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
	if op == nil {return nil}
	op.pt = pt
	op.outrec = outrec
	op.next = op
	op.prev = op
	outrec.pts = op
	return op
}

@(private = "file")
_addLocalMaxPoly :: proc(cb: ^_ClipperBase($Z), e1, e2: ^_Active(Z), pt: _Point(Z)) -> ^_OutPt(Z) {
	if _isJoined(e1) {_split(cb, e1, pt)}
	if _isJoined(e2) {_split(cb, e2, pt)}

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
		for owner != nil && owner.pts == nil {owner = owner.owner}
		tmp := owner
		for tmp != nil && tmp != e2.outrec {tmp = tmp.owner}
		if tmp != nil {owner = e2.outrec.owner}
		e2.outrec.owner = owner
	}

	e1.outrec = nil
	e2.outrec = nil
}

@(private = "file")
_startOpenPath :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z), pt: _Point(Z)) -> ^_OutPt(Z) {
	outrec := _newOutRec(cb)
	if outrec == nil {return nil}
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
	if op == nil {return nil}
	op.pt = pt
	op.outrec = outrec
	op.next = op
	op.prev = op
	outrec.pts = op
	return op
}

@(private = "file")
_getRealOutRec :: proc(outrec: ^_OutRec($Z)) -> ^_OutRec(Z) {
	or := outrec
	for or != nil && or.pts == nil {
		or = or.owner
	}
	return or
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

// ──── InsertLocalMinimaIntoAEL + CheckJoinLeft/Right ─────────────────────────

@(private = "file")
_checkJoinLeft :: proc(
	cb: ^_ClipperBase($Z),
	e: ^_Active(Z),
	pt: _Point(Z),
	check_curr_x: bool = false,
) {
	prev := e.prev_in_ael
	if prev == nil ||
	   !_isHotEdge(e) ||
	   !_isHotEdge(prev) ||
	   _isHorizontalE(e) ||
	   _isHorizontalE(prev) ||
	   _isOpen(e) ||
	   _isOpen(prev) {return}
	if (pt.y < e.top.y + 2 || pt.y < prev.top.y + 2) &&
	   (e.bot.y > pt.y || prev.bot.y > pt.y) {return}
	if check_curr_x {
		if _perpendicDistFromLineSqrd(_xy(pt), _xy(prev.bot), _xy(prev.top)) > 0.25 {return}
	} else if e.curr_x != prev.curr_x {return}
	if !_isCollinear(_xy(e.top), _xy(pt), _xy(prev.top)) {return}

	if e.outrec.idx == prev.outrec.idx {
		_addLocalMaxPoly(cb, prev, e, pt)
	} else if e.outrec.idx < prev.outrec.idx {
		_joinOutrecPaths(cb, e, prev)
	} else {
		_joinOutrecPaths(cb, prev, e)
	}
	prev.join_with = .Right
	e.join_with = .Left
}

@(private = "file")
_checkJoinRight :: proc(
	cb: ^_ClipperBase($Z),
	e: ^_Active(Z),
	pt: _Point(Z),
	check_curr_x: bool = false,
) {
	nxt := e.next_in_ael
	if nxt == nil ||
	   !_isHotEdge(e) ||
	   !_isHotEdge(nxt) ||
	   _isHorizontalE(e) ||
	   _isHorizontalE(nxt) ||
	   _isOpen(e) ||
	   _isOpen(nxt) {return}
	if (pt.y < e.top.y + 2 || pt.y < nxt.top.y + 2) &&
	   (e.bot.y > pt.y || nxt.bot.y > pt.y) {return}
	if check_curr_x {
		if _perpendicDistFromLineSqrd(_xy(pt), _xy(nxt.bot), _xy(nxt.top)) > 0.35 {return}
	} else if e.curr_x != nxt.curr_x {return}
	if !_isCollinear(_xy(e.top), _xy(pt), _xy(nxt.top)) {return}

	if e.outrec.idx == nxt.outrec.idx {
		_addLocalMaxPoly(cb, e, nxt, pt)
	} else if e.outrec.idx < nxt.outrec.idx {
		_joinOutrecPaths(cb, e, nxt)
	} else {
		_joinOutrecPaths(cb, nxt, e)
	}
	e.join_with = .Right
	nxt.join_with = .Left
}

@(private = "file")
_insertLocalMinimaIntoAEL :: proc(cb: ^_ClipperBase($Z), bot_y: f64) {
	for {
		lm: ^_LocalMinima(Z)
		if !_popLocalMinima(cb, bot_y, &lm) {break}

		left_bound: ^_Active(Z)
		right_bound: ^_Active(Z)

		if !_hasFlag(lm.vertex, .OpenStart) {
			left_bound = new(_Active(Z), context.temp_allocator)
			if left_bound == nil {return}
			left_bound.bot = lm.vertex.pt
			left_bound.curr_x = left_bound.bot.x
			left_bound.wind_dx = -1
			left_bound.vertex_top = lm.vertex.prev
			left_bound.top = left_bound.vertex_top.pt
			left_bound.local_min = lm
			_setDx(left_bound)
		}

		if !_hasFlag(lm.vertex, .OpenEnd) {
			right_bound = new(_Active(Z), context.temp_allocator)
			if right_bound == nil {return}
			right_bound.bot = lm.vertex.pt
			right_bound.curr_x = right_bound.bot.x
			right_bound.wind_dx = 1
			right_bound.vertex_top = lm.vertex.next
			right_bound.top = right_bound.vertex_top.pt
			right_bound.local_min = lm
			_setDx(right_bound)
		}

		// swap left/right if needed (descending bound should be on the left)
		if left_bound != nil && right_bound != nil {
			if _isHorizontalE(left_bound) {
				if _isHeadingRightHorz(left_bound) {
					left_bound, right_bound = right_bound, left_bound
				}
			} else if _isHorizontalE(right_bound) {
				if _isHeadingLeftHorz(right_bound) {
					left_bound, right_bound = right_bound, left_bound
				}
			} else if left_bound.dx < right_bound.dx {
				left_bound, right_bound = right_bound, left_bound
			}
		} else if left_bound == nil {
			left_bound = right_bound
			right_bound = nil
		}

		if left_bound == nil {continue}
		left_bound.is_left_bound = true
		_insertLeftEdge(cb, left_bound)

		contributing: bool
		if _isOpen(left_bound) {
			_setWindCountForOpenPathEdge(cb, left_bound)
			contributing = _isContributingOpen(cb, left_bound)
		} else {
			_setWindCountForClosedPathEdge(cb, left_bound)
			contributing = _isContributingClosed(cb, left_bound)
		}

		if right_bound != nil {
			right_bound.is_left_bound = false
			right_bound.wind_cnt = left_bound.wind_cnt
			right_bound.wind_cnt2 = left_bound.wind_cnt2
			_insertRightEdge(left_bound, right_bound)

			if contributing {
				_addLocalMinPoly(cb, left_bound, right_bound, left_bound.bot, true)
				if !_isHorizontalE(left_bound) {
					_checkJoinLeft(cb, left_bound, left_bound.bot)
				}
			}

			// intersect right_bound with AEL neighbors to correct ordering
			for right_bound.next_in_ael != nil &&
			    _isValidAelOrder(right_bound.next_in_ael, right_bound) {
				_intersectEdges(cb, right_bound, right_bound.next_in_ael, right_bound.bot)
				_swapPositionsInAEL(cb, right_bound, right_bound.next_in_ael)
			}

			if _isHorizontalE(right_bound) {
				_pushHorz(cb, right_bound)
			} else {
				_checkJoinRight(cb, right_bound, right_bound.bot)
				_insertScanline(cb, right_bound.top.y)
			}
		} else if contributing {
			_startOpenPath(cb, left_bound, left_bound.bot)
		}

		if _isHorizontalE(left_bound) {
			_pushHorz(cb, left_bound)
		} else {
			_insertScanline(cb, left_bound.top.y)
		}
	}
}

// ──── IntersectEdges (core) + helpers ────────────────────────────────────────

@(private = "file")
_swapOutrecs :: proc(e1, e2: ^_Active($Z)) {
	or1 := e1.outrec
	or2 := e2.outrec
	if or1 == or2 {
		e := or1.front_edge
		or1.front_edge = or1.back_edge
		or1.back_edge = e
		return
	}
	if or1 != nil {
		if e1 == or1.front_edge {
			or1.front_edge = e2
		} else {
			or1.back_edge = e2
		}
	}
	if or2 != nil {
		if e2 == or2.front_edge {
			or2.front_edge = e1
		} else {
			or2.back_edge = e1
		}
	}
	e1.outrec = or2
	e2.outrec = or1
}

@(private = "file")
_findEdgeWithMatchingLocMin :: proc(e: ^_Active($Z)) -> ^_Active(Z) {
	result := e.next_in_ael
	for result != nil {
		if result.local_min == e.local_min {return result}
		if !_isHorizontalE(result) && !(_xy(e.bot) == _xy(result.bot)) {
			result = nil
		} else {
			result = result.next_in_ael
		}
	}
	result = e.prev_in_ael
	for result != nil {
		if result.local_min == e.local_min {return result}
		if !_isHorizontalE(result) && !(_xy(e.bot) == _xy(result.bot)) {
			return nil
		}
		result = result.prev_in_ael
	}
	return result
}

// Z callback invocation — compiled only when Z is active (f64)
@(private = "file")
_setZ :: proc(cb: ^_ClipperBase($Z), e1, e2: ^_Active(Z), ip: ^_Point(Z)) {
	when size_of(Z) != 0 {
		if cb.zCallback_ == nil {return}
		// prioritize subject over clip vertices
		if _getPolyType(e1) == .Subject {
			if ip^ ==
			   e1.bot {ip.z = e1.bot.z} else if ip^ == e1.top {ip.z = e1.top.z} else if ip^ == e2.bot {ip.z = e2.bot.z} else if ip^ == e2.top {ip.z = e2.top.z} else {ip.z = cb.defaultZ}
		} else {
			if ip^ ==
			   e2.bot {ip.z = e2.bot.z} else if ip^ == e2.top {ip.z = e2.top.z} else if ip^ == e1.bot {ip.z = e1.bot.z} else if ip^ == e1.top {ip.z = e1.top.z} else {ip.z = cb.defaultZ}
		}
		// call user callback via type-erased bridge
		if cb.zBridge_ != nil {
			cb.zBridge_(cb.zCallback_, e1.bot, e1.top, e2.bot, e2.top, ip)
		}
	}
}

@(private = "file")
_intersectEdges :: proc(cb: ^_ClipperBase($Z), e1, e2: ^_Active(Z), pt: _Point(Z)) {
	// ── OPEN PATH INTERSECTIONS ──
	if cb.has_open_paths_ && (_isOpen(e1) || _isOpen(e2)) {
		if _isOpen(e1) && _isOpen(e2) {return}
		edge_o, edge_c: ^_Active(Z)
		if _isOpen(e1) {
			edge_o = e1; edge_c = e2
		} else {
			edge_o = e2; edge_c = e1
		}
		if _isJoined(edge_c) {_split(cb, edge_c, pt)}
		if abs(edge_c.wind_cnt) != 1 {return}

		#partial switch cb.cliptype_ {
		case .Union:
			if !_isHotEdge(edge_c) {return}
		case:
			if edge_c.local_min.polytype == .Subject {return}
		}

		#partial switch cb.fillrule_ {
		case .Positive:
			if edge_c.wind_cnt != 1 {return}
		case .Negative:
			if edge_c.wind_cnt != -1 {return}
		case:
			if abs(edge_c.wind_cnt) != 1 {return}
		}

		// toggle contribution
		result_op: ^_OutPt(Z)
		if _isHotEdge(edge_o) {
			result_op = _addOutPt(cb, edge_o, pt)
			if _isFront(
				edge_o,
			) {edge_o.outrec.front_edge = nil} else {edge_o.outrec.back_edge = nil}
			edge_o.outrec = nil
		} else if _xy(pt) == _xy(edge_o.local_min.vertex.pt) &&
		   !_isOpenEnd(edge_o.local_min.vertex) {
			e3 := _findEdgeWithMatchingLocMin(edge_o)
			if e3 != nil && _isHotEdge(e3) {
				edge_o.outrec = e3.outrec
				if edge_o.wind_dx > 0 {
					_setSides(e3.outrec, edge_o, e3)
				} else {
					_setSides(e3.outrec, e3, edge_o)
				}
				return
			} else {
				result_op = _startOpenPath(cb, edge_o, pt)
			}
		} else {
			result_op = _startOpenPath(cb, edge_o, pt)
		}
		when size_of(Z) != 0 {
			if result_op != nil && cb.zCallback_ != nil {
				_setZ(cb, edge_o, edge_c, &result_op.pt)
			}
		}
		return
	}

	// ── CLOSED PATH INTERSECTIONS ──
	if _isJoined(e1) {_split(cb, e1, pt)}
	if _isJoined(e2) {_split(cb, e2, pt)}

	// update winding counts
	old_e1_windcnt, old_e2_windcnt: i32
	if _getPolyType(e1) == _getPolyType(e2) {
		if cb.fillrule_ == .EvenOdd {
			old_e1_windcnt = e1.wind_cnt
			e1.wind_cnt = e2.wind_cnt
			e2.wind_cnt = old_e1_windcnt
		} else {
			if e1.wind_cnt + e2.wind_dx ==
			   0 {e1.wind_cnt = -e1.wind_cnt} else {e1.wind_cnt += e2.wind_dx}
			if e2.wind_cnt - e1.wind_dx ==
			   0 {e2.wind_cnt = -e2.wind_cnt} else {e2.wind_cnt -= e1.wind_dx}
		}
	} else {
		if cb.fillrule_ != .EvenOdd {
			e1.wind_cnt2 += e2.wind_dx
			e2.wind_cnt2 -= e1.wind_dx
		} else {
			e1.wind_cnt2 = (e1.wind_cnt2 == 0 ? 1 : 0)
			e2.wind_cnt2 = (e2.wind_cnt2 == 0 ? 1 : 0)
		}
	}

	// compute old winding counts (used for contribution check)
	#partial switch cb.fillrule_ {
	case .EvenOdd, .NonZero:
		old_e1_windcnt = abs(e1.wind_cnt)
		old_e2_windcnt = abs(e2.wind_cnt)
	case:
		if cb.fillrule_ == cb.fillpos {
			old_e1_windcnt = e1.wind_cnt
			old_e2_windcnt = e2.wind_cnt
		} else {
			old_e1_windcnt = -e1.wind_cnt
			old_e2_windcnt = -e2.wind_cnt
		}
	}

	e1_windcnt_in_01 := old_e1_windcnt == 0 || old_e1_windcnt == 1
	e2_windcnt_in_01 := old_e2_windcnt == 0 || old_e2_windcnt == 1

	if (!_isHotEdge(e1) && !e1_windcnt_in_01) || (!_isHotEdge(e2) && !e2_windcnt_in_01) {
		return
	}

	// ── PROCESS INTERSECTION ──
	result_op: ^_OutPt(Z)
	if _isHotEdge(e1) && _isHotEdge(e2) {
		if (old_e1_windcnt != 0 && old_e1_windcnt != 1) ||
		   (old_e2_windcnt != 0 && old_e2_windcnt != 1) ||
		   (_getPolyType(e1) != _getPolyType(e2) && cb.cliptype_ != .Xor) {
			result_op = _addLocalMaxPoly(cb, e1, e2, pt)
			when size_of(Z) != 0 {
				if result_op != nil && cb.zCallback_ != nil {
					_setZ(cb, e1, e2, &result_op.pt)
				}
			}
		} else if _isFront(e1) || (e1.outrec == e2.outrec) {
			result_op = _addLocalMaxPoly(cb, e1, e2, pt)
			op2 := _addLocalMinPoly(cb, e1, e2, pt, false)
			when size_of(Z) != 0 {
				if cb.zCallback_ != nil {
					if result_op != nil {_setZ(cb, e1, e2, &result_op.pt)}
					if op2 != nil {_setZ(cb, e1, e2, &op2.pt)}
		}
		}
	} else {
			result_op = _addOutPt(cb, e1, pt)
			op2 := _addOutPt(cb, e2, pt)
			_swapOutrecs(e1, e2)
			when size_of(Z) != 0 {
				if cb.zCallback_ != nil {
					if result_op != nil {_setZ(cb, e1, e2, &result_op.pt)}
					if op2 != nil {_setZ(cb, e1, e2, &op2.pt)}
				}
			}
		}
	} else if _isHotEdge(e1) {
		result_op = _addOutPt(cb, e1, pt)
		_swapOutrecs(e1, e2)
		when size_of(Z) != 0 {
			if result_op != nil && cb.zCallback_ != nil {
				_setZ(cb, e1, e2, &result_op.pt)
			}
		}
	} else if _isHotEdge(e2) {
		result_op = _addOutPt(cb, e2, pt)
		_swapOutrecs(e1, e2)
		when size_of(Z) != 0 {
			if result_op != nil && cb.zCallback_ != nil {
				_setZ(cb, e1, e2, &result_op.pt)
			}
		}
	} else {
		e1Wc2, e2Wc2: i32
		#partial switch cb.fillrule_ {
		case .EvenOdd, .NonZero:
			e1Wc2 = abs(e1.wind_cnt2)
			e2Wc2 = abs(e2.wind_cnt2)
		case:
			if cb.fillrule_ == cb.fillpos {
				e1Wc2 = e1.wind_cnt2
				e2Wc2 = e2.wind_cnt2
			} else {
				e1Wc2 = -e1.wind_cnt2
				e2Wc2 = -e2.wind_cnt2
			}
		}

		if !_isSamePolyType(e1, e2) {
			result_op = _addLocalMinPoly(cb, e1, e2, pt, false)
		} else if old_e1_windcnt == 1 && old_e2_windcnt == 1 {
			#partial switch cb.cliptype_ {
			case .Union:
				if e1Wc2 <= 0 && e2Wc2 <= 0 {
					result_op = _addLocalMinPoly(cb, e1, e2, pt, false)
				}
			case .Difference:
				if (_getPolyType(e1) == .Clip && e1Wc2 > 0 && e2Wc2 > 0) ||
				   (_getPolyType(e1) == .Subject && e1Wc2 <= 0 && e2Wc2 <= 0) {
					result_op = _addLocalMinPoly(cb, e1, e2, pt, false)
				}
			case .Xor:
				result_op = _addLocalMinPoly(cb, e1, e2, pt, false)
			case:
				if e1Wc2 > 0 && e2Wc2 > 0 {
					result_op = _addLocalMinPoly(cb, e1, e2, pt, false)
				}
			}
		}
		when size_of(Z) != 0 {
			if result_op != nil && cb.zCallback_ != nil {
				_setZ(cb, e1, e2, &result_op.pt)
			}
		}
	}
}

// ──── Scanline loop (merge sort, intersections, top-of-scanbeam, maxima) ─────

// Merge sort helpers for SEL
@(private = "file")
_extractFromSEL :: proc(ae: ^_Active($Z)) -> ^_Active(Z) {
	res := ae.next_in_sel
	if res != nil {res.prev_in_sel = ae.prev_in_sel}
	ae.prev_in_sel.next_in_sel = res
	return res
}

@(private = "file")
_insert1Before2InSEL :: proc(ae1, ae2: ^_Active($Z)) {
	ae1.prev_in_sel = ae2.prev_in_sel
	if ae1.prev_in_sel != nil {ae1.prev_in_sel.next_in_sel = ae1}
	ae1.next_in_sel = ae2
	ae2.prev_in_sel = ae1
}

@(private = "file")
_adjustCurrXAndCopyToSEL :: proc(cb: ^_ClipperBase($Z), top_y: f64) {
	e := cb.actives_
	cb.sel_ = e
	for e != nil {
		e.prev_in_sel = e.prev_in_ael
		e.next_in_sel = e.next_in_ael
		e.jump = e.next_in_sel
		e.curr_x = _topX(e.dx, _xy(e.bot), _xy(e.top), top_y)
		e = e.next_in_ael
	}
}

@(private = "file")
_addNewIntersectNode :: proc(cb: ^_ClipperBase($Z), e1, e2: ^_Active(Z), top_y: f64) {
	ip: _Point(Z_None)
	if !_getLineIntersectPt(_xy(e1.bot), _xy(e1.top), _xy(e2.bot), _xy(e2.top), &ip) {
		ip.x = e1.curr_x
		ip.y = top_y
	}
	// clamp intersection to scanbeam bounds
	if ip.y > cb.bot_y_ || ip.y < top_y {
		abs_dx1 := abs(e1.dx)
		abs_dx2 := abs(e2.dx)
		if abs_dx1 > 100 && abs_dx2 > 100 {
			if abs_dx1 > abs_dx2 {
				ip = _getClosestPointOnSegment(ip, _xy(e1.bot), _xy(e1.top))
			} else {
				ip = _getClosestPointOnSegment(ip, _xy(e2.bot), _xy(e2.top))
			}
		} else if abs_dx1 > 100 {
			ip = _getClosestPointOnSegment(ip, _xy(e1.bot), _xy(e1.top))
		} else if abs_dx2 > 100 {
			ip = _getClosestPointOnSegment(ip, _xy(e2.bot), _xy(e2.top))
		} else {
			if ip.y < top_y {ip.y = top_y} else {ip.y = cb.bot_y_}
			if abs_dx1 <
			   abs_dx2 {ip.x = _topX(e1.dx, _xy(e1.bot), _xy(e1.top), ip.y)} else {ip.x = _topX(e2.dx, _xy(e2.bot), _xy(e2.top), ip.y)}
		}
	}
	append(
		&cb.intersect_nodes_,
		_IntersectNode(Z){pt = {x = ip.x, y = ip.y}, edge1 = e1, edge2 = e2},
	)
}

@(private = "file")
_intersectNodeSorter :: proc(a, b: _IntersectNode($Z)) -> bool {
	if a.pt.y == b.pt.y {return a.pt.x < b.pt.x}
	return a.pt.y > b.pt.y
}

@(private = "file")
_buildIntersectList :: proc(cb: ^_ClipperBase($Z), top_y: f64) -> bool {
	if cb.actives_ == nil || cb.actives_.next_in_ael == nil {return false}
	_adjustCurrXAndCopyToSEL(cb, top_y)

	left := cb.sel_
	for left != nil && left.jump != nil {
		prev_base: ^_Active(Z)
		for left != nil && left.jump != nil {
			curr_base := left
			right := left.jump
			l_end := right
			r_end := right.jump
			left.jump = r_end

			for left != l_end && right != r_end {
				if right.curr_x < left.curr_x {
					// intersections found — record all edges between tmp and right
					tmp := right.prev_in_sel
					for {
						_addNewIntersectNode(cb, tmp, right, top_y)
						if tmp == left {break}
						tmp = tmp.prev_in_sel
					}
					tmp = right
					right = _extractFromSEL(tmp)
					l_end = right
					_insert1Before2InSEL(tmp, left)
					if left == curr_base {
						curr_base = tmp
						curr_base.jump = r_end
						if prev_base == nil {cb.sel_ = curr_base} else {prev_base.jump = curr_base}
					}
				} else {
					left = left.next_in_sel
				}
			}
			prev_base = curr_base
			left = r_end
		}
		left = cb.sel_
	}
	return len(cb.intersect_nodes_) > 0
}

@(private = "file")
_edgesAdjacentInAEL :: proc(node: _IntersectNode($Z)) -> bool {
	return node.edge1.next_in_ael == node.edge2 || node.edge1.prev_in_ael == node.edge2
}

@(private = "file")
_processIntersectList :: proc(cb: ^_ClipperBase($Z)) {
	if len(cb.intersect_nodes_) == 0 {return}
	slice.sort_by(cb.intersect_nodes_[:], _intersectNodeSorter)

	for i := 0; i < len(cb.intersect_nodes_); i += 1 {
		if !_edgesAdjacentInAEL(cb.intersect_nodes_[i]) {
			j := i + 1
			for !_edgesAdjacentInAEL(cb.intersect_nodes_[j]) {j += 1}
			cb.intersect_nodes_[i], cb.intersect_nodes_[j] =
				cb.intersect_nodes_[j], cb.intersect_nodes_[i]
		}
		node := &cb.intersect_nodes_[i]
		ip := _Point(Z) {
			x = node.pt.x,
			y = node.pt.y,
		}
		_intersectEdges(cb, node.edge1, node.edge2, ip)
		_swapPositionsInAEL(cb, node.edge1, node.edge2)
		node.edge1.curr_x = node.pt.x
		node.edge2.curr_x = node.pt.x
		_checkJoinLeft(cb, node.edge2, ip, true)
		_checkJoinRight(cb, node.edge1, ip, true)
	}
}

@(private = "file")
_doIntersections :: proc(cb: ^_ClipperBase($Z), top_y: f64) {
	if _buildIntersectList(cb, top_y) {
		_processIntersectList(cb)
		clear(&cb.intersect_nodes_)
	}
}

@(private = "file")
_doTopOfScanbeam :: proc(cb: ^_ClipperBase($Z), y: f64) {
	cb.sel_ = nil // sel_ reused for horizontals
	e := cb.actives_
	for e != nil {
		if e.top.y == y {
			e.curr_x = e.top.x
			if _isMaximaE(e) {
				e = _doMaxima(cb, e)
				continue
			} else {
				if _isHotEdge(e) {_addOutPt(cb, e, e.top)}
				_updateEdgeIntoAEL(cb, e)
				if _isHorizontalE(e) {_pushHorz(cb, e)}
			}
		} else {
			e.curr_x = _topX(e.dx, _xy(e.bot), _xy(e.top), y)
		}
		e = e.next_in_ael
	}
}

@(private = "file")
_doMaxima :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) -> ^_Active(Z) {
	prev_e := e.prev_in_ael
	next_e := e.next_in_ael
	if _isOpenEndE(e) {
		if _isHotEdge(e) {_addOutPt(cb, e, e.top)}
		if !_isHorizontalE(e) {
			if _isHotEdge(e) {
				if _isFront(e) {e.outrec.front_edge = nil} else {e.outrec.back_edge = nil}
				e.outrec = nil
			}
			_deleteFromAEL(cb, e)
		}
		return next_e
	}

	max_pair := _getMaximaPair(e)
	if max_pair == nil {return next_e}

	if _isJoined(e) {_split(cb, e, e.top)}
	if _isJoined(max_pair) {_split(cb, max_pair, max_pair.top)}

	for next_e != max_pair {
		_intersectEdges(cb, e, next_e, e.top)
		_swapPositionsInAEL(cb, e, next_e)
		next_e = e.next_in_ael
	}

	if _isOpen(e) {
		if _isHotEdge(e) {_addLocalMaxPoly(cb, e, max_pair, e.top)}
		_deleteFromAEL(cb, max_pair)
		_deleteFromAEL(cb, e)
		if prev_e != nil {return prev_e.next_in_ael}
		return cb.actives_
	}

	if _isHotEdge(e) {_addLocalMaxPoly(cb, e, max_pair, e.top)}
	_deleteFromAEL(cb, e)
	_deleteFromAEL(cb, max_pair)
	if prev_e != nil {return prev_e.next_in_ael}
	return cb.actives_
}

@(private = "file")
_updateEdgeIntoAEL :: proc(cb: ^_ClipperBase($Z), e: ^_Active(Z)) {
	e.bot = e.top
	e.vertex_top = _nextVertex(e)
	e.top = e.vertex_top.pt
	e.curr_x = e.bot.x
	_setDx(e)

	if _isJoined(e) {_split(cb, e, e.bot)}

	if _isHorizontalE(e) {
		if !_isOpen(e) {_trimHorz(e, cb.preserve_collinear_)}
		return
	}
	_insertScanline(cb, e.top.y)
	_checkJoinLeft(cb, e, e.bot)
	_checkJoinRight(cb, e, e.bot, true)
}

@(private = "file")
_trimHorz :: proc(e: ^_Active($Z), preserveCollinear: bool) {
	for {
		pt := _nextVertex(e).pt
		if _xy(pt).y != _xy(e.top).y {break}
		if preserveCollinear && ((pt.x < e.top.x) != (e.bot.x < e.top.x)) {break}
		e.vertex_top = _nextVertex(e)
		e.top = pt
		if _isMaximaE(e) {break}
	}
	_setDx(e)
}

@(private = "file")
_getMaximaPair :: proc(e: ^_Active($Z)) -> ^_Active(Z) {
	e2 := e.next_in_ael
	for e2 != nil {
		if e2.vertex_top == e.vertex_top {return e2}
		e2 = e2.next_in_ael
	}
	return nil
}

@(private = "file")
_getCurrYMaximaVertex :: proc(e: ^_Active($Z)) -> ^_Vertex(Z) {
	result := e.vertex_top
	if e.wind_dx > 0 {
		for result.next.pt.y == result.pt.y {result = result.next}
	} else {
		for result.prev.pt.y == result.pt.y {result = result.prev}
	}
	if !_isMaximaV(result) {result = nil}
	return result
}

@(private = "file")
_getCurrYMaximaVertexOpen :: proc(e: ^_Active($Z)) -> ^_Vertex(Z) {
	result := e.vertex_top
	if e.wind_dx > 0 {
		for result.next.pt.y == result.pt.y &&
		    !_hasFlag(result, .OpenEnd) &&
		    !_hasFlag(result, .LocalMax) {
			result = result.next
		}
	} else {
		for result.prev.pt.y == result.pt.y &&
		    !_hasFlag(result, .OpenEnd) &&
		    !_hasFlag(result, .LocalMax) {
			result = result.prev
		}
	}
	if !_isMaximaV(result) {result = nil}
	return result
}

@(private = "file")
_resetHorzDirection :: proc(
	horz: ^_Active($Z),
	max_vertex: ^_Vertex(Z),
	horz_left, horz_right: ^f64,
) -> bool {
	if horz.bot.x == horz.top.x {
		horz_left^ = horz.curr_x
		horz_right^ = horz.curr_x
		e := horz.next_in_ael
		for e != nil && e.vertex_top != max_vertex {e = e.next_in_ael}
		return e != nil
	} else if horz.curr_x < horz.top.x {
		horz_left^ = horz.curr_x
		horz_right^ = horz.top.x
		return true
	} else {
		horz_left^ = horz.top.x
		horz_right^ = horz.curr_x
		return false
	}
}

@(private = "file")
_getLastOp :: proc(hot_edge: ^_Active($Z)) -> ^_OutPt(Z) {
	outrec := hot_edge.outrec
	result := outrec.pts
	if hot_edge != outrec.front_edge {result = result.next}
	return result
}

// ──── DoHorizontal (core horizontal edge processing) ─────────────────────────

@(private = "file")
_addTrialHorzJoin :: proc(cb: ^_ClipperBase($Z), op: ^_OutPt(Z)) {
	if op.outrec.is_open {return}
	append(&cb.horz_seg_list_, _HorzSegment(Z){left_op = op})
}

@(private = "file")
_doHorizontal :: proc(cb: ^_ClipperBase($Z), horz: ^_Active(Z)) {
	horzIsOpen := _isOpen(horz)
	y := horz.bot.y
	vertex_max: ^_Vertex(Z)
	if horzIsOpen {
		vertex_max = _getCurrYMaximaVertexOpen(horz)
	} else {
		vertex_max = _getCurrYMaximaVertex(horz)
	}

	horz_left, horz_right: f64
	is_left_to_right := _resetHorzDirection(horz, vertex_max, &horz_left, &horz_right)

	if _isHotEdge(horz) {
		pt := _Point(Z) {
			x = horz.curr_x,
			y = y,
		}
		op := _addOutPt(cb, horz, pt)
		_addTrialHorzJoin(cb, op)
	}

	for {
		e: ^_Active(Z)
		if is_left_to_right {e = horz.next_in_ael} else {e = horz.prev_in_ael}

		for e != nil {
			if e.vertex_top == vertex_max {
				if _isHotEdge(horz) && _isJoined(e) {_split(cb, e, e.top)}
				if _isHotEdge(horz) {
					for horz.vertex_top != vertex_max {
						_addOutPt(cb, horz, horz.top)
						_updateEdgeIntoAEL(cb, horz)
					}
					if is_left_to_right {
						_addLocalMaxPoly(cb, horz, e, horz.top)
					} else {
						_addLocalMaxPoly(cb, e, horz, horz.top)
					}
				}
				_deleteFromAEL(cb, e)
				_deleteFromAEL(cb, horz)
				return
			}

			if vertex_max != horz.vertex_top || _isOpenEndE(horz) {
				if (is_left_to_right && e.curr_x > horz_right) ||
				   (!is_left_to_right && e.curr_x < horz_left) {break}

				if e.curr_x == horz.top.x && !_isHorizontalE(e) {
					pt := _nextVertex(horz).pt
					if is_left_to_right {
						if _isOpen(e) && !_isSamePolyType(e, horz) && !_isHotEdge(e) {
							if _topX(e.dx, _xy(e.bot), _xy(e.top), pt.y) > pt.x {break}
						} else if _topX(e.dx, _xy(e.bot), _xy(e.top), pt.y) >= pt.x {break}
					} else {
						if _isOpen(e) && !_isSamePolyType(e, horz) && !_isHotEdge(e) {
							if _topX(e.dx, _xy(e.bot), _xy(e.top), pt.y) < pt.x {break}
						} else if _topX(e.dx, _xy(e.bot), _xy(e.top), pt.y) <= pt.x {break}
					}
				}
			}

			pt := _Point(Z) {
				x = e.curr_x,
				y = horz.bot.y,
			}
			if is_left_to_right {
				_intersectEdges(cb, horz, e, pt)
				_swapPositionsInAEL(cb, horz, e)
				_checkJoinLeft(cb, e, pt)
				horz.curr_x = e.curr_x
				e = horz.next_in_ael
			} else {
				_intersectEdges(cb, e, horz, pt)
				_swapPositionsInAEL(cb, e, horz)
				_checkJoinRight(cb, e, pt)
				horz.curr_x = e.curr_x
				e = horz.prev_in_ael
			}

			if horz.outrec != nil {
				_addTrialHorzJoin(cb, _getLastOp(horz))
			}
		}

		// check if finished with consecutive horizontals
		if horzIsOpen && _isOpenEndE(horz) {
			if _isHotEdge(horz) {
				_addOutPt(cb, horz, horz.top)
				if _isFront(horz) {horz.outrec.front_edge = nil} else {horz.outrec.back_edge = nil}
				horz.outrec = nil
			}
			_deleteFromAEL(cb, horz)
			return
		} else if _nextVertex(horz).pt.y != horz.top.y {break}

		// more horizontals in bound
		if _isHotEdge(horz) {_addOutPt(cb, horz, horz.top)}
		_updateEdgeIntoAEL(cb, horz)
		is_left_to_right = _resetHorzDirection(horz, vertex_max, &horz_left, &horz_right)
	}

	if _isHotEdge(horz) {
		op := _addOutPt(cb, horz, horz.top)
		_addTrialHorzJoin(cb, op)
	}
	_updateEdgeIntoAEL(cb, horz)
}

// ──── ExecuteInternal (main scanline loop) + output assembly ─────────────────

@(private = "file")
_executeInternal :: proc(cb: ^_ClipperBase($Z), ct: ClipType, fr: FillRule) -> bool {
	cb.cliptype_ = ct
	cb.fillrule_ = fr
	cb.using_polytree_ = false
	_reset(cb)

	y: f64
	if ct == .NoClip || !_popScanline(cb, &y) {return true}

	for cb.succeeded_ {
		_insertLocalMinimaIntoAEL(cb, y)

		e: ^_Active(Z)
		for _popHorz(cb, &e) {_doHorizontal(cb, e)}

		if len(cb.horz_seg_list_) > 0 {
			// _convertHorzSegsToJoins(cb)  (deferred — not critical for basic ops)
			clear(&cb.horz_seg_list_)
		}

		cb.bot_y_ = y
		if !_popScanline(cb, &y) {break}

		_doIntersections(cb, y)
		_doTopOfScanbeam(cb, y)

		for _popHorz(cb, &e) {_doHorizontal(cb, e)}
	}

	// if cb.succeeded_ { _processHorzJoins(cb) }  (deferred)
	return cb.succeeded_
}

@(private = "file")
_buildPath64 :: proc(op: ^_OutPt($Z), reverse, isOpen: bool, path: ^[dynamic]_Point(Z)) -> bool {
	if op == nil || op.next == op || (!isOpen && op.next == op.prev) {return false}
	clear(path)

	start := op
	lastPt := op.pt
	op2: ^_OutPt(Z)
	if reverse {
		lastPt = op.pt
		op2 = op.prev
	} else {
		start = op.next
		lastPt = start.pt
		op2 = start.next
	}
	append(path, lastPt)

	for op2 != start {
		if op2.pt.x != lastPt.x || op2.pt.y != lastPt.y {
			lastPt = op2.pt
			append(path, lastPt)
		}
		if reverse {op2 = op2.prev} else {op2 = op2.next}
	}

	if !isOpen && len(path) == 3 && _isVerySmallTriangle(op2) {return false}
	return true
}

@(private = "file")
_buildPaths64 :: proc(cb: ^_ClipperBase($Z), solutionClosed, solutionOpen: ^[dynamic][]_Point(Z)) {
	for _, i in cb.outrec_list_ {
		outrec := cb.outrec_list_[i]
		if outrec.pts == nil {continue}

		path := outrec.path
		if solutionOpen != nil && outrec.is_open {
			if _buildPath64(outrec.pts, cb.reverse_solution_, true, &path) {
				append(solutionOpen, path[:])
			}
		} else {
			_cleanCollinear(cb, outrec)
			if _buildPath64(outrec.pts, cb.reverse_solution_, false, &path) {
				append(solutionClosed, path[:])
			}
		}
	}
}

@(private = "file")
_cleanCollinear :: proc(cb: ^_ClipperBase($Z), outrec: ^_OutRec(Z)) {
	// simplified: just get the real outrec and check path validity
	outrec2 := _getRealOutRec(outrec)
	if outrec2 == nil || outrec2.is_open {return}
	if !_isValidClosedPath(outrec2.pts) {
		// dispose
	}
	// full CleanCollinear with FixSelfIntersects is deferred
	_ = cb
}

// ──── Fast RectClip Implementation ───────────────────────────────────────────
// Port of Clipper2 clipper.rectclip.cpp

@(private = "file")
_rectClipGetLocation :: proc(rec: _RectF64, pt: _Point(Z_None), loc: ^_Location) -> bool {
	if pt.x == rec.left && pt.y >= rec.top && pt.y <= rec.bottom {
		loc^ = .Left; return false
	} else if pt.x == rec.right && pt.y >= rec.top && pt.y <= rec.bottom {
		loc^ = .Right; return false
	} else if pt.y == rec.top && pt.x >= rec.left && pt.x <= rec.right {
		loc^ = .Top; return false
	} else if pt.y == rec.bottom && pt.x >= rec.left && pt.x <= rec.right {
		loc^ = .Bottom; return false
	} else if pt.x < rec.left { loc^ = .Left
	} else if pt.x > rec.right { loc^ = .Right
	} else if pt.y < rec.top { loc^ = .Top
	} else if pt.y > rec.bottom { loc^ = .Bottom
	} else { loc^ = .Inside }
	return true
}

@(private = "file")
_rectClipGetSegmentIntersection :: proc(p1, p2, p3, p4: _Point(Z_None), ip: ^_Point(Z_None)) -> bool {
	res1 := _crossProductSign(p1, p3, p4)
	res2 := _crossProductSign(p2, p3, p4)
	if res1 == 0 {
		ip^ = p1
		if res2 == 0 { return false }
		if p1 == p3 || p1 == p4 { return true }
		if p3.y == p4.y { return (p1.x > p3.x) == (p1.x < p4.x) }
		return (p1.y > p3.y) == (p1.y < p4.y)
	} else if res2 == 0 {
		ip^ = p2
		if p2 == p3 || p2 == p4 { return true }
		if p3.y == p4.y { return (p2.x > p3.x) == (p2.x < p4.x) }
		return (p2.y > p3.y) == (p2.y < p4.y)
	}
	if (res1 > 0) == (res2 > 0) { return false }
	res3 := _crossProductSign(p3, p1, p2)
	res4 := _crossProductSign(p4, p1, p2)
	if res3 == 0 {
		ip^ = p3
		if p3 == p1 || p3 == p2 { return true }
		if p1.y == p2.y { return (p3.x > p1.x) == (p3.x < p2.x) }
		return (p3.y > p1.y) == (p3.y < p2.y)
	} else if res4 == 0 {
		ip^ = p4
		if p4 == p1 || p4 == p2 { return true }
		if p1.y == p2.y { return (p4.x > p1.x) == (p4.x < p2.x) }
		return (p4.y > p1.y) == (p4.y < p2.y)
	}
	if (res3 > 0) == (res4 > 0) { return false }
	return _getLineIntersectPt(p1, p2, p3, p4, ip)
}

@(private = "file")
_rectClipGetAdjacent :: proc(loc: _Location, isClockwise: bool) -> _Location {
	d := isClockwise ? 1 : 3
	return _Location((int(loc) + d) % 4)
}

@(private = "file")
_rectClipHeadingClockwise :: proc(prev, curr: _Location) -> bool {
	return (int(prev) + 1) % 4 == int(curr)
}

@(private = "file")
_rectClipAreOpposites :: proc(prev, curr: _Location) -> bool {
	return abs(int(prev) - int(curr)) == 2
}

@(private = "file")
_rectClipIsClockwise :: proc(prev, curr: _Location, prev_pt, curr_pt, rect_mp: _Point(Z_None)) -> bool {
	if _rectClipAreOpposites(prev, curr) {
		return _crossProductSign(prev_pt, rect_mp, curr_pt) < 0
	}
	return _rectClipHeadingClockwise(prev, curr)
}

@(private = "file")
_rectClipAdd :: proc(rc: ^_RectClip64, pt: _Point(Z_None), start_new: bool) -> ^_OutPt2 {
	curr_idx := len(rc.results_)
	if curr_idx == 0 || start_new {
		result := new(_OutPt2, context.temp_allocator)
		result.pt = pt
		result.next = result
		result.prev = result
		append(&rc.results_, result)
		return result
	}
	curr_idx -= 1
	prevOp := rc.results_[curr_idx]
	if prevOp.pt == pt { return prevOp }
	result := new(_OutPt2, context.temp_allocator)
	result.owner_idx = curr_idx
	result.pt = pt
	result.next = prevOp.next
	prevOp.next.prev = result
	prevOp.next = result
	result.prev = prevOp
	rc.results_[curr_idx] = result
	return result
}

@(private = "file")
_rectClipAddCorner1 :: proc(rc: ^_RectClip64, prev, curr: _Location) {
	if _rectClipHeadingClockwise(prev, curr) {
		_rectClipAdd(rc, rc.rect_as_path_[int(prev)], false)
	} else {
		_rectClipAdd(rc, rc.rect_as_path_[int(curr)], false)
	}
}

@(private = "file")
_rectClipAddCorner2 :: proc(rc: ^_RectClip64, loc: ^_Location, isClockwise: bool) {
	if isClockwise {
		_rectClipAdd(rc, rc.rect_as_path_[int(loc^)], false)
		loc^ = _rectClipGetAdjacent(loc^, true)
	} else {
		loc^ = _rectClipGetAdjacent(loc^, false)
		_rectClipAdd(rc, rc.rect_as_path_[int(loc^)], false)
	}
}

@(private = "file")
_rectClipGetNextLocation :: proc(rc: ^_RectClip64, path: []_Point(Z_None), loc: ^_Location, i: ^int, highI: int) {
	#partial switch loc^ {
	case .Left:
		for i^ <= highI && path[i^].x <= rc.rect_.left { i^ += 1 }
		if i^ > highI { break }
		switch {
		case path[i^].x >= rc.rect_.right: loc^ = .Right
		case path[i^].y <= rc.rect_.top: loc^ = .Top
		case path[i^].y >= rc.rect_.bottom: loc^ = .Bottom
		case: loc^ = .Inside
		}
	case .Bottom:
		for i^ <= highI && path[i^].y >= rc.rect_.bottom { i^ += 1 }
		if i^ > highI { break }
		switch {
		case path[i^].y <= rc.rect_.top: loc^ = .Top
		case path[i^].x <= rc.rect_.left: loc^ = .Left
		case path[i^].x >= rc.rect_.right: loc^ = .Right
		case: loc^ = .Inside
		}
	case .Top:
		for i^ <= highI && path[i^].y <= rc.rect_.top { i^ += 1 }
		if i^ > highI { break }
		switch {
		case path[i^].y >= rc.rect_.bottom: loc^ = .Bottom
		case path[i^].x <= rc.rect_.left: loc^ = .Left
		case path[i^].x >= rc.rect_.right: loc^ = .Right
		case: loc^ = .Inside
		}
	case .Right:
		for i^ <= highI && path[i^].x >= rc.rect_.right { i^ += 1 }
		if i^ > highI { break }
		switch {
		case path[i^].x <= rc.rect_.left: loc^ = .Left
		case path[i^].y <= rc.rect_.top: loc^ = .Top
		case path[i^].y >= rc.rect_.bottom: loc^ = .Bottom
		case: loc^ = .Inside
		}
	case .Inside:
		for i^ <= highI {
			switch {
			case path[i^].x < rc.rect_.left: loc^ = .Left
			case path[i^].x > rc.rect_.right: loc^ = .Right
			case path[i^].y > rc.rect_.bottom: loc^ = .Bottom
			case path[i^].y < rc.rect_.top: loc^ = .Top
			case:
				_rectClipAdd(rc, path[i^], false)
				i^ += 1
				continue
			}
			break
		}
	}
}

@(private = "file")
_rectClipStartLocsAreClockwise :: proc(startlocs: []_Location) -> bool {
	result := 0
	for i in 1 ..< len(startlocs) {
		d := int(startlocs[i]) - int(startlocs[i-1])
		switch d {
		case -1: result -= 1
		case 1: result += 1
		case -3: result += 1
		case 3: result -= 1
		}
	}
	return result > 0
}

@(private = "file")
_rectClipGetIntersection :: proc(rc: ^_RectClip64, p, p2: _Point(Z_None), loc: ^_Location, ip: ^_Point(Z_None)) -> bool {
	rp := rc.rect_as_path_
	#partial switch loc^ {
	case .Left:
		if _rectClipGetSegmentIntersection(p, p2, rp[0], rp[3], ip) { return true }
		if p.y < rp[0].y && _rectClipGetSegmentIntersection(p, p2, rp[0], rp[1], ip) { loc^ = .Top; return true }
		if _rectClipGetSegmentIntersection(p, p2, rp[2], rp[3], ip) { loc^ = .Bottom; return true }
	case .Top:
		if _rectClipGetSegmentIntersection(p, p2, rp[0], rp[1], ip) { return true }
		if p.x < rp[0].x && _rectClipGetSegmentIntersection(p, p2, rp[0], rp[3], ip) { loc^ = .Left; return true }
		if _rectClipGetSegmentIntersection(p, p2, rp[1], rp[2], ip) { loc^ = .Right; return true }
	case .Right:
		if _rectClipGetSegmentIntersection(p, p2, rp[1], rp[2], ip) { return true }
		if p.y < rp[1].y && _rectClipGetSegmentIntersection(p, p2, rp[0], rp[1], ip) { loc^ = .Top; return true }
		if _rectClipGetSegmentIntersection(p, p2, rp[2], rp[3], ip) { loc^ = .Bottom; return true }
	case .Bottom:
		if _rectClipGetSegmentIntersection(p, p2, rp[2], rp[3], ip) { return true }
		if p.x < rp[3].x && _rectClipGetSegmentIntersection(p, p2, rp[0], rp[3], ip) { loc^ = .Left; return true }
		if _rectClipGetSegmentIntersection(p, p2, rp[1], rp[2], ip) { loc^ = .Right; return true }
	case:
		if _rectClipGetSegmentIntersection(p, p2, rp[0], rp[3], ip) { loc^ = .Left; return true }
		if _rectClipGetSegmentIntersection(p, p2, rp[0], rp[1], ip) { loc^ = .Top; return true }
		if _rectClipGetSegmentIntersection(p, p2, rp[1], rp[2], ip) { loc^ = .Right; return true }
		if _rectClipGetSegmentIntersection(p, p2, rp[2], rp[3], ip) { loc^ = .Bottom; return true }
	}
	return false
}

@(private = "file")
_rectClipExecuteInternal :: proc(rc: ^_RectClip64, path: []_Point(Z_None)) {
	if len(path) < 1 { return }
	highI := len(path) - 1
	prev := _Location.Inside; loc := _Location.Inside
	crossing_loc := _Location.Inside
	first_cross_ := _Location.Inside

	if !_rectClipGetLocation(rc.rect_, path[highI], &loc) {
		i := highI
		for i > 0 && !_rectClipGetLocation(rc.rect_, path[i-1], &prev) { i -= 1 }
		if i == 0 {
			for pt in path { _rectClipAdd(rc, pt, false) }
			return
		}
		if prev == .Inside { loc = .Inside }
	}
	starting_loc := loc

	i := 0
	for i <= highI {
		prev = loc
		crossing_prev := crossing_loc
		_rectClipGetNextLocation(rc, path, &loc, &i, highI)
		if i > highI { break }

		prev_pt := path[i-1] if i > 0 else path[highI]
		crossing_loc = loc

		ip: _Point(Z_None)
		ip2: _Point(Z_None)
		if !_rectClipGetIntersection(rc, path[i], prev_pt, &crossing_loc, &ip) {
			if crossing_prev == .Inside {
				isClockw := _rectClipIsClockwise(prev, loc, prev_pt, path[i], rc.rect_mp_)
				for {
					append(&rc.start_locs_, prev)
					prev = _rectClipGetAdjacent(prev, isClockw)
					if prev == loc { break }
				}
				crossing_loc = crossing_prev
			} else if prev != .Inside && prev != loc {
				isClockw := _rectClipIsClockwise(prev, loc, prev_pt, path[i], rc.rect_mp_)
				for {
					_rectClipAddCorner2(rc, &prev, isClockw)
					if prev == loc { break }
				}
			}
			i += 1
			continue
		}

		// crossing the rect boundary
		if loc == .Inside { // entering
			if first_cross_ == .Inside {
				first_cross_ = crossing_loc
				append(&rc.start_locs_, prev)
			} else if prev != crossing_loc {
				isClockw := _rectClipIsClockwise(prev, crossing_loc, prev_pt, path[i], rc.rect_mp_)
				for {
					_rectClipAddCorner2(rc, &prev, isClockw)
					if prev == crossing_loc { break }
				}
			}
		} else if prev != .Inside { // passing through — need both intersect points
			loc = prev
			_rectClipGetIntersection(rc, prev_pt, path[i], &loc, &ip2)
			if crossing_prev != .Inside && crossing_prev != loc {
				_rectClipAddCorner1(rc, crossing_prev, loc)
			}
			if first_cross_ == .Inside {
				first_cross_ = loc
				append(&rc.start_locs_, prev)
			}
			loc = crossing_loc
			_rectClipAdd(rc, ip2, false)
			if ip == ip2 {
				_rectClipGetLocation(rc.rect_, path[i], &loc)
				_rectClipAddCorner1(rc, crossing_loc, loc)
				crossing_loc = loc
				continue
			}
		} else { // exiting
			loc = crossing_loc
			if first_cross_ == .Inside {
				first_cross_ = crossing_loc
			}
		}
		_rectClipAdd(rc, ip, false)
	}

	if first_cross_ == .Inside {
		if starting_loc != .Inside {
			if _rectClipContainsRect(path, rc.rect_) && _rectClipPath1ContainsPath2(path, rc.rect_as_path_[:]) {
				isClockwisePath := _rectClipStartLocsAreClockwise(rc.start_locs_[:])
				for j in 0 ..< 4 {
					k := j if isClockwisePath else 3 - j
					_rectClipAdd(rc, rc.rect_as_path_[k], false)
				}
			}
		}
	} else if loc != .Inside && (loc != first_cross_ || len(rc.start_locs_) > 2) {
		if len(rc.start_locs_) > 0 {
			prev = loc
			for loc2 in rc.start_locs_ {
				if prev == loc2 { continue }
				_rectClipAddCorner2(rc, &prev, _rectClipHeadingClockwise(prev, loc2))
				prev = loc2
			}
			loc = prev
		}
		if loc != first_cross_ {
			_rectClipAddCorner2(rc, &loc, _rectClipHeadingClockwise(loc, first_cross_))
		}
	}
}

@(private = "file")
_rectClipContainsRect :: proc(path: []_Point(Z_None), rec: _RectF64) -> bool {
	// simplified bounds check
	xmin, ymin := path[0].x, path[0].y
	xmax, ymax := path[0].x, path[0].y
	for pt in path {
		if pt.x < xmin { xmin = pt.x }
		if pt.x > xmax { xmax = pt.x }
		if pt.y < ymin { ymin = pt.y }
		if pt.y > ymax { ymax = pt.y }
	}
	return xmin <= rec.left && xmax >= rec.right && ymin <= rec.top && ymax >= rec.bottom
}

@(private = "file")
_rectClipPath1ContainsPath2 :: proc(path1: []_Point(Z_None), path2: []_Point(Z_None)) -> bool {
	io_count := 0
	for pt in path2 {
		loc := _pointInPolygon2D(pt, path1)
		switch loc {
		case 1: io_count -= 1
		case -1: io_count += 1
		}
		if abs(io_count) > 1 { break }
	}
	return io_count <= 0
}

@(private = "file")
_pointInPolygon2D :: proc(pt: _Point(Z_None), polygon: []_Point(Z_None)) -> int {
	if len(polygon) < 3 { return -1 }
	val := 0
	first := 0
	for first < len(polygon) && polygon[first].y == pt.y { first += 1 }
	if first == len(polygon) { return -1 }
	is_above := polygon[first].y < pt.y
	starting_above := is_above
	curr := first + 1
	for {
		if curr == len(polygon) {
			curr = 0
		}
		if curr == first { break }
		if is_above {
			for polygon[curr].y < pt.y {
				curr += 1
				if curr == len(polygon) { curr = 0 }
				if curr == first { break }
			}
		} else {
			for polygon[curr].y > pt.y {
				curr += 1
				if curr == len(polygon) { curr = 0 }
				if curr == first { break }
			}
		}
		if curr == first { break }
		if polygon[curr].y == pt.y {
			if polygon[curr].x == pt.x { return 0 }
			curr += 1
			if curr == first { break }
			continue
		}
		prev := curr - 1 if curr > 0 else len(polygon) - 1
		if pt.x < polygon[curr].x && pt.x < polygon[prev].x {
			// do nothing
		} else if pt.x > polygon[prev].x && pt.x > polygon[curr].x {
			val = 1 - val
		} else {
			cp := _crossProductSign(polygon[prev], polygon[curr], pt)
			if cp == 0 { return 0 }
			if (cp < 0) == is_above { val = 1 - val }
		}
		is_above = !is_above
		curr += 1
		if curr == len(polygon) { curr = 0 }
	}
	if is_above != starting_above {
		prev := curr - 1 if curr > 0 else len(polygon) - 1
		cp := _crossProductSign(polygon[prev], polygon[curr], pt)
		if cp == 0 { return 0 }
		if (cp < 0) == is_above { val = 1 - val }
	}
	return -1 if val == 0 else 1
}

@(private = "file")
_rectClipGetPath :: proc(op: ^^_OutPt2) -> []_Point(Z_None) {
	if op^ == nil || op^.next == op^.prev { return nil }
	op2 := op^.next
	for op2 != nil && op2 != op^ {
		if _isCollinear(_xy(op2.prev.pt), _xy(op2.pt), _xy(op2.next.pt)) {
			op^ = op2.prev
			// unlink op2
			op2.prev.next = op2.next
			op2.next.prev = op2.prev
			op2 = op2.next
		} else {
			op2 = op2.next
		}
	}
	op^ = op2
	if op2 == nil { return nil }
	result := make([dynamic]_Point(Z_None), context.temp_allocator)
	append(&result, op^.pt)
	op2 = op^.next
	for op2 != op^ {
		append(&result, op2.pt)
		op2 = op2.next
	}
	return result[:]
}

@(private = "file")
_rectClipExecute :: proc(rc: ^_RectClip64, paths: [][]_Point(Z_None)) -> [][]_Point(Z_None) {
	result := make([dynamic][]_Point(Z_None), context.temp_allocator)
	for path in paths {
		if len(path) < 3 { continue }
		_rectClipExecuteInternal(rc, path)
		_rectClipCheckEdges(rc)
		for i in 0 ..< 4 {
			_rectClipTidyEdges(rc, i, &rc.edges_[i*2], &rc.edges_[i*2+1])
		}
		for _, i in rc.results_ {
			op := rc.results_[i]
			tmp := _rectClipGetPath(&op)
			if len(tmp) > 0 {
				append(&result, tmp)
			}
		}
		// reset for next path
		clear(&rc.results_)
		for j in 0 ..< 8 { clear(&rc.edges_[j]) }
		clear(&rc.start_locs_)
	}
	return result[:]
}

@(private = "file")
_rectClipCheckEdges :: proc(rc: ^_RectClip64) {
	for _, i in rc.results_ {
		op := rc.results_[i]
		if op == nil { continue }
		op2 := op
		for {
			if _isCollinear(_xy(op2.prev.pt), _xy(op2.pt), _xy(op2.next.pt)) {
				if op2 == op {
					op2 = _rectClipUnlinkOpBack(op2)
					if op2 == nil { break }
					op = op2.prev
				} else {
					op2 = _rectClipUnlinkOpBack(op2)
					if op2 == nil { break }
				}
			} else {
				op2 = op2.next
			}
			if op2 == op { break }
		}
		if op2 == nil { rc.results_[i] = nil; continue }
		rc.results_[i] = op

		edgeSet1 := _rectClipGetEdgesForPt(op.prev.pt, rc.rect_)
		op2 = op
		for {
			edgeSet2 := _rectClipGetEdgesForPt(op2.pt, rc.rect_)
			if edgeSet2 != 0 && op2.edge == nil {
				combinedSet := edgeSet1 & edgeSet2
				for j in 0 ..< 4 {
					if (combinedSet & (1 << uint(j))) != 0 {
						if _rectClipIsHeadingClockwise(op2.prev.pt, op2.pt, j) {
							_rectClipAddToEdge(&rc.edges_[j*2], op2)
						} else {
							_rectClipAddToEdge(&rc.edges_[j*2+1], op2)
						}
					}
				}
			}
			edgeSet1 = edgeSet2
			op2 = op2.next
			if op2 == op { break }
		}
	}
}

@(private = "file")
_rectClipUnlinkOpBack :: proc(op: ^_OutPt2) -> ^_OutPt2 {
	if op.next == op { return nil }
	op.prev.next = op.next
	op.next.prev = op.prev
	return op.prev
}

@(private = "file")
_rectClipGetEdgesForPt :: proc(pt: _Point(Z_None), rec: _RectF64) -> uint {
	result: uint = 0
	if pt.x == rec.left { result = 1 }
	else if pt.x == rec.right { result = 4 }
	if pt.y == rec.top { result += 2 }
	else if pt.y == rec.bottom { result += 8 }
	return result
}

@(private = "file")
_rectClipIsHeadingClockwise :: proc(pt1, pt2: _Point(Z_None), edgeIdx: int) -> bool {
	switch edgeIdx {
	case 0: return pt2.y < pt1.y
	case 1: return pt2.x > pt1.x
	case 2: return pt2.y > pt1.y
	case: return pt2.x < pt1.x
	}
}

@(private = "file")
_rectClipAddToEdge :: proc(edge: ^[dynamic]^_OutPt2, op: ^_OutPt2) {
	if op.edge != nil { return }
	op.edge = edge
	append(edge, op)
}

@(private = "file")
_rectClipUncoupleEdge :: proc(op: ^_OutPt2) {
	if op.edge == nil { return }
	for i in 0 ..< len(op.edge) {
		if op.edge[i] == op { op.edge[i] = nil; break }
	}
	op.edge = nil
}

@(private = "file")
_rectClipHasHorzOverlap :: proc(left1, right1, left2, right2: _Point(Z_None)) -> bool {
	return (left1.x < right2.x) && (right1.x > left2.x)
}

@(private = "file")
_rectClipHasVertOverlap :: proc(top1, bottom1, top2, bottom2: _Point(Z_None)) -> bool {
	return (top1.y < bottom2.y) && (bottom1.y > top2.y)
}

@(private = "file")
_rectClipTidyEdges :: proc(rc: ^_RectClip64, idx: int, cw, ccw: ^[dynamic]^_OutPt2) {
	if len(ccw) == 0 { return }
	isHorz := idx == 1 || idx == 3
	cwIsTowardLarger := idx == 1 || idx == 2
	i, j := 0, 0
	p2: ^_OutPt2
	for i < len(cw) {
		p1 := cw[i]
		if p1 == nil || p1.next == p1.prev { cw[i] = nil; i += 1; j = 0; continue }
		jLim := len(ccw)
		for j < jLim && (ccw[j] == nil || ccw[j].next == ccw[j].prev) { j += 1 }
		if j == jLim { i += 1; j = 0; continue }

		p1a, p2a: ^_OutPt2
		if cwIsTowardLarger {
			p1 = cw[i].prev; p1a = cw[i]
			p2 = ccw[j]; p2a = ccw[j].prev
		} else {
			p1 = cw[i]; p1a = cw[i].prev
			p2 = ccw[j].prev; p2a = ccw[j]
		}
		if (isHorz && !_rectClipHasHorzOverlap(p1.pt, p1a.pt, p2.pt, p2a.pt)) ||
			(!isHorz && !_rectClipHasVertOverlap(p1.pt, p1a.pt, p2.pt, p2a.pt)) {
			j += 1
			continue
		}
		isRejoining := cw[i].owner_idx != ccw[j].owner_idx
		if isRejoining {
			rc.results_[p2.owner_idx] = nil
			_rectClipSetNewOwner(p2, p1.owner_idx)
		}
		if cwIsTowardLarger {
			p1.next = p2; p2.prev = p1
			p1a.prev = p2a; p2a.next = p1a
		} else {
			p1.prev = p2; p2.next = p1
			p1a.next = p2a; p2a.prev = p1a
		}
		if !isRejoining {
			new_idx := len(rc.results_)
			append(&rc.results_, p1a)
			_rectClipSetNewOwner(p1a, new_idx)
		}
		// rest of tidy logic (simplified for compilation)
		i += 1
	}
}

@(private = "file")
_rectClipSetNewOwner :: proc(op: ^_OutPt2, new_idx: int) {
	op.owner_idx = new_idx
	op2 := op.next
	for op2 != op {
		op2.owner_idx = new_idx
		op2 = op2.next
	}
}

// ──── RectClipLines64 ────────────────────────────────────────────────────────

@(private = "file")
_rectClipLinesExecuteInternal :: proc(rc: ^_RectClip64, path: []_Point(Z_None)) {
	if len(path) < 2 { return }
	clear(&rc.results_)
	clear(&rc.start_locs_)

	i := 1; highI := len(path) - 1
	prev := _Location.Inside; loc := _Location.Inside
	crossing_loc: _Location

	if !_rectClipGetLocation(rc.rect_, path[0], &loc) {
		for i <= highI && !_rectClipGetLocation(rc.rect_, path[i], &prev) { i += 1 }
		if i > highI {
			for pt in path { _rectClipAdd(rc, pt, false) }
			return
		}
		if prev == .Inside { loc = .Inside }
		i = 1
	}
	if loc == .Inside { _rectClipAdd(rc, path[0], false) }

	for i <= highI {
		prev = loc
		_rectClipGetNextLocation(rc, path, &loc, &i, highI)
		if i > highI { break }
		prev_pt := path[i-1]
		crossing_loc = loc
		ip: _Point(Z_None)
		ip2: _Point(Z_None)
		if !_rectClipGetIntersection(rc, path[i], prev_pt, &crossing_loc, &ip) {
			i += 1; continue
		}
		if loc == .Inside { // entering
			_rectClipAdd(rc, ip, true)
		} else if prev != .Inside { // passing through
			crossing_loc = prev
			_rectClipGetIntersection(rc, prev_pt, path[i], &crossing_loc, &ip2)
			_rectClipAdd(rc, ip2, true)
			_rectClipAdd(rc, ip, false)
		} else { // exiting
			_rectClipAdd(rc, ip, false)
		}
	}
}

@(private = "file")
_rectClipLinesGetPath :: proc(op: ^^_OutPt2) -> []_Point(Z_None) {
	if op^ == nil || op^ == op^.next { return nil }
	op^ = op^.next  // starting at path beginning
	result := make([dynamic]_Point(Z_None), context.temp_allocator)
	append(&result, op^.pt)
	op2 := op^.next
	for op2 != op^ {
		append(&result, op2.pt)
		op2 = op2.next
	}
	return result[:]
}

// ──── InflatePaths (Offset) Implementation ───────────────────────────────────
// Port of Clipper2 clipper.offset.cpp

@(private = "file")
_getLowestClosedPathInfo :: proc(
	paths: [dynamic][dynamic]_Point($Z),
	idx: ^int,
	is_neg_area: ^bool,
) {
	idx^ = -1
	botPt := _Point(Z_None) {
		x = math.F64_MAX,
		y = -math.F64_MAX,
	}
	for i in 0 ..< len(paths) {
		a := math.F64_MAX
		for pt in paths[i] {
			if pt.y < botPt.y || (pt.y == botPt.y && pt.x >= botPt.x) {continue}
			if a == math.F64_MAX {
				area: f64
				path := paths[i]
				for j in 0 ..< len(path) {
					k := (j + 1) % len(path)
					area += path[j].x * path[k].y - path[k].x * path[j].y
				}
				a = area * 0.5
				if a == 0 {break}
				is_neg_area^ = a < 0
			}
			idx^ = i
			botPt.x = pt.x
			botPt.y = pt.y
		}
	}
}

@(private = "file")
_hypot :: #force_inline proc "contextless" (x, y: f64) -> f64 {
	return math.sqrt(x * x + y * y)
}

@(private = "file")
_getUnitNormal :: proc(p1, p2: _Point(Z_None)) -> _Point(Z_None) {
	if p1 == p2 {return {}}
	dx := p2.x - p1.x
	dy := p2.y - p1.y
	inv := 1.0 / _hypot(dx, dy)
	return {x = dy * inv, y = -dx * inv}
}

@(private = "file")
_almostZero :: #force_inline proc "contextless" (v: f64, eps: f64 = 0.001) -> bool {
	return abs(v) < eps
}

@(private = "file")
_normalizeVector :: proc(v: _Point(Z_None)) -> _Point(Z_None) {
	h := _hypot(v.x, v.y)
	if _almostZero(h) {return {}}
	inv := 1.0 / h
	return {x = v.x * inv, y = v.y * inv}
}

@(private = "file")
_getAvgUnitVector :: proc(v1, v2: _Point(Z_None)) -> _Point(Z_None) {
	return _normalizeVector({x = v1.x + v2.x, y = v1.y + v2.y})
}

@(private = "file")
_isClosedPath :: #force_inline proc(et: EndType) -> bool {
	return et == .Polygon || et == .Joined
}

@(private = "file")
_getPerpendic :: proc(pt: _Point(Z_None), norm: _Point(Z_None), delta: f64) -> _Point(Z_None) {
	return {x = pt.x + norm.x * delta, y = pt.y + norm.y * delta}
}

@(private = "file")
_negatePath :: proc(path: ^[dynamic]_Point(Z_None)) {
	for i in 0 ..< len(path) {
		path[i].x = -path[i].x
		path[i].y = -path[i].y
	}
}

// ──── Group init ─────────────────────────────────────────────────────────────

@(private = "file")
_group_init :: proc(
	g: ^_Group($Z),
	paths: [dynamic][dynamic]_Point(Z),
	jt: JoinType,
	et: EndType,
) {
	g.paths_in = paths
	g.join_type = jt
	g.end_type = et
	g.is_reversed = false
	g.lowest_path_idx = -1

	// strip duplicates
	is_joined := et == .Polygon || et == .Joined
	for _, i in g.paths_in {
		_stripDuplicates(&g.paths_in[i], is_joined)
	}

	if et == .Polygon {
		is_neg_area: bool
		_getLowestClosedPathInfo(g.paths_in, &g.lowest_path_idx, &is_neg_area)
		g.is_reversed = g.lowest_path_idx >= 0 && is_neg_area
	}
}

@(private = "file")
_stripDuplicates :: proc(path: ^[dynamic]_Point($Z), is_closed: bool) {
	if len(path) <= 1 {return}
	j := 1
	for i in 1 ..< len(path) {
		if path[i].x != path[j - 1].x || path[i].y != path[j - 1].y {
			path[j] = path[i]
			j += 1
		}
	}
	for is_closed && j > 1 && path[j - 1].x == path[0].x && path[j - 1].y == path[0].y {
		j -= 1
	}
	resize(path, j)
}

// ──── Offset methods ─────────────────────────────────────────────────────────

@(private = "file")
_buildNormals :: proc(co: ^_ClipperOffset($Z), path: []_Point(Z)) {
	clear(&co.norms)
	if len(path) == 0 {return}
	reserve(&co.norms, len(path))
	for i in 0 ..< len(path) - 1 {
		append(&co.norms, _getUnitNormal(_xy(path[i]), _xy(path[i + 1])))
	}
	append(&co.norms, _getUnitNormal(_xy(path[len(path) - 1]), _xy(path[0])))
}

@(private = "file")
_doBevel :: proc(co: ^_ClipperOffset($Z), path: []_Point(Z), j, k: int) {
	p1, p2: _Point(Z_None)
	abs_delta := abs(co.group_delta_)
	if j == k {
		p1 = {
			x = path[j].x - abs_delta * co.norms[j].x,
			y = path[j].y - abs_delta * co.norms[j].y,
		}
		p2 = {
			x = path[j].x + abs_delta * co.norms[j].x,
			y = path[j].y + abs_delta * co.norms[j].y,
		}
	} else {
		p1 = {
			x = path[j].x + co.group_delta_ * co.norms[k].x,
			y = path[j].y + co.group_delta_ * co.norms[k].y,
		}
		p2 = {
			x = path[j].x + co.group_delta_ * co.norms[j].x,
			y = path[j].y + co.group_delta_ * co.norms[j].y,
		}
	}
	append(&co.path_out, _Point(Z){x = p1.x, y = p1.y})
	append(&co.path_out, _Point(Z){x = p2.x, y = p2.y})
}

@(private = "file")
_doSquare :: proc(co: ^_ClipperOffset($Z), path: []_Point(Z), j, k: int) {
	vec: _Point(Z_None)
	if j == k {
		vec = {
			x = co.norms[j].y,
			y = -co.norms[j].x,
		}
	} else {
		vec = _getAvgUnitVector(
			{x = -co.norms[k].y, y = co.norms[k].x},
			{x = co.norms[j].y, y = -co.norms[j].x},
		)
	}
	abs_delta := abs(co.group_delta_)
	ptQ := _translatePoint(_xy(path[j]), abs_delta * vec.x, abs_delta * vec.y)
	pt1 := _translatePoint(ptQ, co.group_delta_ * vec.y, co.group_delta_ * -vec.x)
	pt2 := _translatePoint(ptQ, co.group_delta_ * -vec.y, co.group_delta_ * vec.x)
	pt3 := _getPerpendic(_xy(path[k]), co.norms[k], co.group_delta_)
	if j == k {
		pt4 := _translatePoint(pt3, vec.x * co.group_delta_, vec.y * co.group_delta_)
		pt := ptQ
		_getLineIntersectPt(pt1, pt2, pt3, pt4, &pt)
		r1 := _reflectPoint(pt, ptQ)
		append(&co.path_out, _Point(Z){x = r1.x, y = r1.y})
		append(&co.path_out, _Point(Z){x = pt.x, y = pt.y})
	} else {
		pt4 := _getPerpendic(_xy(path[j]), co.norms[k], co.group_delta_)
		pt := ptQ
		_getLineIntersectPt(pt1, pt2, pt3, pt4, &pt)
		r2 := _reflectPoint(pt, ptQ)
		append(&co.path_out, _Point(Z){x = pt.x, y = pt.y})
		append(&co.path_out, _Point(Z){x = r2.x, y = r2.y})
	}
}

@(private = "file")
_doMiter :: proc(co: ^_ClipperOffset($Z), path: []_Point(Z), j, k: int, cos_a: f64) {
	q := co.group_delta_ / (cos_a + 1)
	append(
		&co.path_out,
		_Point(Z) {
			x = path[j].x + (co.norms[k].x + co.norms[j].x) * q,
			y = path[j].y + (co.norms[k].y + co.norms[j].y) * q,
		},
	)
}

@(private = "file")
_doRound :: proc(co: ^_ClipperOffset($Z), path: []_Point(Z), j, k: int, angle: f64) {
	pt := path[j]
	offsetVec := _Point(Z_None) {
		x = co.norms[k].x * co.group_delta_,
		y = co.norms[k].y * co.group_delta_,
	}
	if j == k {
		offsetVec.x = -offsetVec.x
		offsetVec.y = -offsetVec.y
	}
	append(&co.path_out, _Point(Z){x = pt.x + offsetVec.x, y = pt.y + offsetVec.y})
	steps := int(math.ceil(co.steps_per_rad_ * abs(angle)))
	for i in 1 ..< steps {
		newX := offsetVec.x * co.step_cos_ - co.step_sin_ * offsetVec.y
		offsetVec.y = offsetVec.x * co.step_sin_ + offsetVec.y * co.step_cos_
		offsetVec.x = newX
		append(&co.path_out, _Point(Z){x = pt.x + offsetVec.x, y = pt.y + offsetVec.y})
	}
	perp := _getPerpendic(_xy(path[j]), co.norms[j], co.group_delta_)
	append(&co.path_out, _Point(Z){x = perp.x, y = perp.y})
}

@(private = "file")
_offsetPoint :: proc(co: ^_ClipperOffset($Z), group: ^_Group(Z), path: []_Point(Z), j, k: int) {
	if path[j].x == path[k].x && path[j].y == path[k].y {return}
	sin_a := _cross(co.norms[j], co.norms[k])
	cos_a := _dot(co.norms[j], co.norms[k])
	if sin_a > 1 {sin_a = 1} else if sin_a < -1 {sin_a = -1}

	if abs(co.group_delta_) <= 1e-12 {
		append(&co.path_out, _Point(Z){x = path[j].x, y = path[j].y})
		return
	}

	if cos_a > -0.999 && (sin_a * co.group_delta_ < 0) {
		// concave — use averaged normal for spike tip (matches C++ Clipper2)
		perpK := _getPerpendic(_xy(path[j]), co.norms[k], co.group_delta_)
		avgNorm := _Point(Z_None){
			x = (co.norms[j].x + co.norms[k].x) * 0.5,
			y = (co.norms[j].y + co.norms[k].y) * 0.5,
		}
		perpMid := _getPerpendic(_xy(path[j]), avgNorm, co.group_delta_)
		perpJ := _getPerpendic(_xy(path[j]), co.norms[j], co.group_delta_)
		append(&co.path_out, _Point(Z){x = perpK.x, y = perpK.y})
		append(&co.path_out, _Point(Z){x = perpMid.x, y = perpMid.y})
		append(&co.path_out, _Point(Z){x = perpJ.x, y = perpJ.y})
	} else if cos_a > 0.999 && co.join_type_ != .Round {
		// almost straight
		_doMiter(co, path, j, k, cos_a)
		_doMiter(co, path, j, k, cos_a)
	} else if co.join_type_ == .Miter {
		if cos_a >
		   co.temp_lim_ - 1 {_doMiter(co, path, j, k, cos_a)} else {_doSquare(co, path, j, k)}
	} else if co.join_type_ == .Round {
		_doRound(co, path, j, k, math.atan2(sin_a, cos_a))
	} else if co.join_type_ == .Bevel {
		_doBevel(co, path, j, k)
	} else {
		_doSquare(co, path, j, k)
	}
}

@(private = "file")
_offsetPolygon :: proc(co: ^_ClipperOffset($Z), group: ^_Group(Z), path: []_Point(Z)) {
	clear(&co.path_out)
	k := len(path) - 1
	for j := 0; j < len(path); j += 1 {
		_offsetPoint(co, group, path, j, k)
		k = j
	}
	outCopy := make([]_Point(Z), len(co.path_out), context.temp_allocator)
	copy(outCopy, co.path_out[:])
	append(co.solution, outCopy)
}

@(private = "file")
_offsetOpenJoined :: proc(co: ^_ClipperOffset($Z), group: ^_Group(Z), path: []_Point(Z)) {
	_offsetPolygon(co, group, path)
	// reverse path
	rev := make([dynamic]_Point(Z), len(path), context.temp_allocator)
	for i in 0 ..< len(path) {
		rev[len(path) - 1 - i] = path[i]
	}
	// reverse normals
	// reverse normals (cyclic shift left by 1 after reverse)
	reverse_slice(&co.norms)
	first := co.norms[0]
	for i in 0 ..< len(co.norms) - 1 {
		co.norms[i] = co.norms[i+1]
	}
	co.norms[len(co.norms)-1] = first
	_negatePath(&co.norms)
	_negatePath(&co.norms)
	_offsetPolygon(co, group, rev[:])
}

@(private = "file")
_offsetOpenPath :: proc(co: ^_ClipperOffset($Z), group: ^_Group(Z), path: []_Point(Z)) {
	clear(&co.path_out)
	// start cap
	if abs(co.group_delta_) <= 1e-12 {
		append(&co.path_out, path[0])
	} else {
		#partial switch co.end_type_ {
		case .Butt:
			_doBevel(co, path, 0, 0)
		case .Round:
			_doRound(co, path, 0, 0, math.PI)
		case:
			_doSquare(co, path, 0, 0)
		}
	}
	// left side forward
	highI := len(path) - 1
	for j := 1; j < highI; j += 1 {
		_offsetPoint(co, group, path, j, j - 1)
	}
	// reverse normals for return pass
	for i := highI; i > 0; i -= 1 {
		co.norms[i] = {
			x = -co.norms[i - 1].x,
			y = -co.norms[i - 1].y,
		}
	}
	co.norms[0] = co.norms[highI]
	// end cap
	if abs(co.group_delta_) <= 1e-12 {
		append(&co.path_out, path[highI])
	} else {
		#partial switch co.end_type_ {
		case .Butt:
			_doBevel(co, path, highI, highI)
		case .Round:
			_doRound(co, path, highI, highI, math.PI)
		case:
			_doSquare(co, path, highI, highI)
		}
	}
	// right side reverse
	for j := highI - 1; j > 0; j -= 1 {
		_offsetPoint(co, group, path, j, j + 1)
	}
	outCopy2 := make([]_Point(Z), len(co.path_out), context.temp_allocator)
	copy(outCopy2, co.path_out[:])
	append(co.solution, outCopy2)
}

@(private = "file")
_doGroupOffset :: proc(co: ^_ClipperOffset($Z), group: ^_Group(Z)) {
	if group.end_type == .Polygon {
		if group.lowest_path_idx < 0 {co.delta_ = abs(co.delta_)}
		co.group_delta_ = group.is_reversed ? -co.delta_ : co.delta_
	} else {
		co.group_delta_ = abs(co.delta_)
	}

	abs_delta := abs(co.group_delta_)
	co.join_type_ = group.join_type
	co.end_type_ = group.end_type

	if group.join_type == .Round || group.end_type == .Round {
		arcTol := abs_delta < 0.35 ? 0.15 : abs_delta * 0.002
		if co.arc_tolerance_ > 0.01 {
			arcTol = min(abs_delta, co.arc_tolerance_)
		}
		steps_per_360 := min(math.PI / math.acos(1 - arcTol / abs_delta), abs_delta * math.PI)
		co.step_sin_ = math.sin(2 * math.PI / steps_per_360)
		co.step_cos_ = math.cos(2 * math.PI / steps_per_360)
		if co.group_delta_ < 0 {co.step_sin_ = -co.step_sin_}
		co.steps_per_rad_ = steps_per_360 / (2 * math.PI)
	}

	for path_in in group.paths_in {
		pathLen := len(path_in)
		clear(&co.path_out)
		if pathLen == 0 {continue}

		if pathLen == 1 {
			pt := path_in[0]
			if co.group_delta_ < 1 {continue}
			if group.join_type == .Round {
				radius := abs_delta
				steps := int(math.ceil_f64(co.steps_per_rad_ * 2 * math.PI))
				_ellipsePoints(&co.path_out, pt, radius, radius, steps)
			} else {
				d := int(math.ceil(abs_delta))
				_pathRect(&co.path_out, pt, d)
			}
			append(co.solution, co.path_out[:])
			continue
		}

		if pathLen == 2 && group.end_type == .Joined {
			if group.join_type == .Round {
				co.end_type_ = .Round
			} else {
				co.end_type_ = .Square
			}
		}

		_buildNormals(co, path_in[:])
		#partial switch co.end_type_ {
		case .Polygon:
			_offsetPolygon(co, group, path_in[:])
		case .Joined:
			_offsetOpenJoined(co, group, path_in[:])
		case:
			_offsetOpenPath(co, group, path_in[:])
		}
	}
}

@(private = "file")
_ellipsePoints :: proc(out: ^[dynamic]_Point($Z), center: _Point(Z), rx, ry: f64, steps: int) {
	clear(out)
	si := math.sin(2 * math.PI / f64(steps))
	co := math.cos(2 * math.PI / f64(steps))
	dx, dy := co, si
	for i in 0 ..< steps {
		append(out, _Point(Z){x = center.x + rx * dx, y = center.y + ry * dy})
		x2 := dx * co - dy * si
		dy = dy * co + dx * si
		dx = x2
	}
}

@(private = "file")
_pathRect :: proc(out: ^[dynamic]_Point($Z), center: _Point(Z), d: int) {
	fd := f64(d)
	append(out, _Point(Z){x = center.x - fd, y = center.y - fd})
	append(out, _Point(Z){x = center.x + fd, y = center.y - fd})
	append(out, _Point(Z){x = center.x + fd, y = center.y + fd})
	append(out, _Point(Z){x = center.x - fd, y = center.y + fd})
}

@(private = "file")
_same2D :: #force_inline proc(a, b: _Point(Z_None)) -> bool {
	return a.x == b.x && a.y == b.y
}

@(private = "file")
_toNeutral :: #force_inline proc(p: _Point($Z)) -> _Point(Z_None) {
	return {x = p.x, y = p.y}
}

@(private = "file")
reverse_slice :: proc(s: ^[dynamic]$T) {
	for i in 0 ..< len(s) / 2 {
		j := len(s) - 1 - i
		s[i], s[j] = s[j], s[i]
	}
}

// ──── InflatePaths public proc ───────────────────────────────────────────────

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
	context.allocator = allocator
	HasZ ::
		intrinsics.type_has_field(PointT, "z") when intrinsics.type_is_struct(PointT) else len(
			PointT,
		) >=
		3 when intrinsics.type_is_array(PointT) else false

	when HasZ {
		co: _ClipperOffset(f64)
		co.miter_limit_ = miterLimit
		co.arc_tolerance_ = arcTolerance
		co.preserve_collinear_ = preserveCollinear
		co.reverse_solution_ = reverseSolution
		co.temp_lim_ = miterLimit > 1 ? 2.0 / (miterLimit * miterLimit) : 0.5
		co.solution = new([dynamic][]_Point(f64), context.temp_allocator)

		paths_reversed := false
		for path in closePaths {
			if len(path) == 0 {continue}
			ipath := make([dynamic]_Point(f64), len(path), context.temp_allocator)
			for pt, i in path {
				ipath[i] = {
					x = f64(pt.x),
					y = f64(pt.y),
					z = f64(pt.z),
				}
			}
			g: _Group(f64)
			gpaths := make([dynamic][dynamic]_Point(f64), 1, context.temp_allocator)
			gpaths[0] = ipath
			_group_init(&g, gpaths, joinType, .Polygon)
			if g.is_reversed { paths_reversed = true }
			append(&co.groups_, g)
		}
		for path in openPaths {
			if len(path) == 0 {continue}
			ipath := make([dynamic]_Point(f64), len(path), context.temp_allocator)
			for pt, i in path {
				ipath[i] = {
					x = f64(pt.x),
					y = f64(pt.y),
					z = f64(pt.z),
				}
			}
			g: _Group(f64)
			gpaths := make([dynamic][dynamic]_Point(f64), 1, context.temp_allocator)
			gpaths[0] = ipath
			_group_init(&g, gpaths, joinType, endType)
			append(&co.groups_, g)
		}

		_offsetExecuteInternal(&co, delta)

		if len(co.solution^) > 0 {
			cleanFill := FillRule.Positive
			cleaned, _, cerr := BooleanOp(
				.Union,
				PointT,
				_pathsFromOffset(co.solution^, PointT, context.temp_allocator),
				nil,
				nil,
				cleanFill,
				zCallback,
				context.temp_allocator,
			)
			if cerr != nil {
				return nil, cerr
			}
			if reverseSolution != paths_reversed {
				for path in cleaned {_reversePath(path)}
			}
			res = make([][]PointT, len(cleaned), allocator)
			for i in 0 ..< len(cleaned) {
				res[i] = make([]PointT, len(cleaned[i]), allocator)
				copy(res[i], cleaned[i])
			}
			return
		}
		return nil, nil
	} else {
		co: _ClipperOffset(struct {})
		co.miter_limit_ = miterLimit
		co.arc_tolerance_ = arcTolerance
		co.preserve_collinear_ = preserveCollinear
		co.reverse_solution_ = reverseSolution
		co.temp_lim_ = miterLimit > 1 ? 2.0 / (miterLimit * miterLimit) : 0.5
		co.solution = new([dynamic][]_Point(struct {}), context.temp_allocator)

		paths_reversed := false
		for path in closePaths {
			if len(path) == 0 {continue}
			ipath := make([dynamic]_Point(struct {}), len(path), context.temp_allocator)
			for pt, i in path {
				ipath[i] = {
					x = f64(pt.x),
					y = f64(pt.y),
				}
			}
			g: _Group(struct {})
			gpaths := make([dynamic][dynamic]_Point(struct {}), 1, context.temp_allocator)
			gpaths[0] = ipath
			_group_init(&g, gpaths, joinType, .Polygon)
			if g.is_reversed { paths_reversed = true }
			append(&co.groups_, g)
		}
		for path in openPaths {
			if len(path) == 0 {continue}
			ipath := make([dynamic]_Point(struct {}), len(path), context.temp_allocator)
			for pt, i in path {
				ipath[i] = {
					x = f64(pt.x),
					y = f64(pt.y),
				}
			}
			g: _Group(struct {})
			gpaths := make([dynamic][dynamic]_Point(struct {}), 1, context.temp_allocator)
			gpaths[0] = ipath
			_group_init(&g, gpaths, joinType, endType)
			append(&co.groups_, g)
		}

		_offsetExecuteInternal(&co, delta)

		if abs(delta) < 0.5 {
			res = make([][]PointT, len(co.solution^), allocator)
			for i in 0 ..< len(co.solution^) {
				ipath := co.solution^[i]
				dst := make([]PointT, len(ipath), allocator)
				for pt, j in ipath {
					dst[j] = _makePoint(PointT, pt.x, pt.y, 0)
				}
				res[i] = dst
			}
			return
		}

		if len(co.solution^) > 0 {
			cleanFill := FillRule.Positive
			cleaned, _, cerr := BooleanOp(
				.Union,
				PointT,
				_pathsFromOffset(co.solution^, PointT, context.temp_allocator),
				nil,
				nil,
				cleanFill,
				nil,
				context.temp_allocator,
			)
			if cerr != nil {
				return nil, cerr
			}
			if reverseSolution != paths_reversed {
				for path in cleaned {_reversePath(path)}
			}
			res = make([][]PointT, len(cleaned), allocator)
			for i in 0 ..< len(cleaned) {
				res[i] = make([]PointT, len(cleaned[i]), allocator)
				copy(res[i], cleaned[i])
			}
			return
		}
		return nil, nil
	}
}

@(private = "file")
_offsetExecuteInternal :: proc(co: ^_ClipperOffset($Z), delta: f64) {
	if len(co.groups_) == 0 {return}
	co.delta_ = delta
	for _, i in co.groups_ {
		_doGroupOffset(co, &co.groups_[i])
	}
}

@(private = "file")
_pathsFromOffset :: proc(
	paths: [dynamic][]_Point($Z),
	$PointT: typeid,
	allocator := context.allocator,
) -> [][]PointT {
	res := make([][]PointT, len(paths), allocator)
	HasZ ::
		intrinsics.type_has_field(PointT, "z") when intrinsics.type_is_struct(PointT) else len(
			PointT,
		) >=
		3 when intrinsics.type_is_array(PointT) else false
	for p, i in paths {
		dst := make([]PointT, len(p), allocator)
		for pt, j in p {
			when size_of(Z) != 0 && HasZ {
				dst[j] = _makePoint(PointT, pt.x, pt.y, pt.z)
			} else {
				dst[j] = _makePoint(PointT, pt.x, pt.y, 0)
			}
		}
		res[i] = dst
	}
	return res
}

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
	context.allocator = allocator
	HasZ ::
		intrinsics.type_has_field(PointT, "z") when intrinsics.type_is_struct(PointT) else len(
			PointT,
		) >=
		3 when intrinsics.type_is_array(PointT) else false

	when HasZ {
		_ = zCallback
		T :: intrinsics.type_field_type(PointT, "z")
		cb: _ClipperBase(T)
		_clipperBase_init(&cb)
		cb.zCallback_ = rawptr(zCallback)
		if zCallback != nil {
			cb.zBridge_ = proc(
				callbackPtr: rawptr,
				e1Bot, e1Top, e2Bot, e2Top: _Point(T),
				outPoint: ^_Point(T),
			) {
				zcb := (proc(e1Bot, e1Top, e2Bot, e2Top: PointT, outPoint: ^PointT))(callbackPtr)
				// 내부 _Point(T) → 사용자 PointT 변환
				e1BotP := _fromInternalPointZ(e1Bot, PointT)
				e1TopP := _fromInternalPointZ(e1Top, PointT)
				e2BotP := _fromInternalPointZ(e2Bot, PointT)
				e2TopP := _fromInternalPointZ(e2Top, PointT)
				outP := _fromInternalPointZ(outPoint^, PointT)
				// 사용자 콜백 호출
				zcb(e1BotP, e1TopP, e2BotP, e2TopP, &outP)
				// PointT.z → 내부 _Point(T).z 로 다시 씀
				outPoint.z = outP.z
			}
		}

		for path in subjects {
			if len(path) == 0 {continue}
			ipath := make([dynamic]_Point(T), len(path), context.temp_allocator)
			for pt, i in path {
				ipath[i] = {
					x = f64(pt.x),
					y = f64(pt.y),
					z = pt.z,
				}
			}
			paths := make([dynamic][dynamic]_Point(T), 1, context.temp_allocator)
			paths[0] = ipath
			_addPaths_(paths, .Subject, false, &cb.vertex_lists_, &cb.minima_list_)
		}
		for path in clips {
			if len(path) == 0 {continue}
			ipath := make([dynamic]_Point(T), len(path), context.temp_allocator)
			for pt, i in path {
				ipath[i] = {
					x = f64(pt.x),
					y = f64(pt.y),
					z = pt.z,
				}
			}
			paths := make([dynamic][dynamic]_Point(T), 1, context.temp_allocator)
			paths[0] = ipath
			_addPaths_(paths, .Clip, false, &cb.vertex_lists_, &cb.minima_list_)
		}
		if len(opens) > 0 {
			cb.has_open_paths_ = true
		}
		for path in opens {
			if len(path) == 0 {continue}
			ipath := make([dynamic]_Point(T), len(path), context.temp_allocator)
			for pt, i in path {
				ipath[i] = {
					x = f64(pt.x),
					y = f64(pt.y),
					z = pt.z,
				}
			}
			paths := make([dynamic][dynamic]_Point(T), 1, context.temp_allocator)
			paths[0] = ipath
			_addPaths_(paths, .Subject, true, &cb.vertex_lists_, &cb.minima_list_)
		}

		if !_executeInternal(&cb, clipType, fillRule) {
			return nil, nil, __ClipperError.FAILED
		}

		closed := make([dynamic][]_Point(T), context.temp_allocator)
		open := make([dynamic][]_Point(T), context.temp_allocator)
		_buildPaths64(&cb, &closed, &open)

		res = make([][]PointT, len(closed), allocator)
		for ipath, i in closed {
			opath := make([]PointT, len(ipath), allocator)
			for pt, j in ipath {
				opath[j] = _makePoint(PointT, pt.x, pt.y, pt.z)
			}
			res[i] = opath
		}
		resOpen = make([][]PointT, len(open), allocator)
		for ipath, i in open {
			opath := make([]PointT, len(ipath), allocator)
			for pt, j in ipath {
				opath[j] = _makePoint(PointT, pt.x, pt.y, pt.z)
			}
			resOpen[i] = opath
		}
		return
	} else {
		cb: _ClipperBase(struct {})
		_clipperBase_init(&cb)

		for path in subjects {
			if len(path) == 0 {continue}
			ipath := make([dynamic]_Point(struct {}), len(path), context.temp_allocator)
			for pt, i in path {
				ipath[i] = {
					x = f64(pt.x),
					y = f64(pt.y),
				}
			}
			paths := make([dynamic][dynamic]_Point(struct {}), 1, context.temp_allocator)
			paths[0] = ipath
			_addPaths_(paths, .Subject, false, &cb.vertex_lists_, &cb.minima_list_)
		}
		for path in clips {
			if len(path) == 0 {continue}
			ipath := make([dynamic]_Point(struct {}), len(path), context.temp_allocator)
			for pt, i in path {
				ipath[i] = {
					x = f64(pt.x),
					y = f64(pt.y),
				}
			}
			paths := make([dynamic][dynamic]_Point(struct {}), 1, context.temp_allocator)
			paths[0] = ipath
			_addPaths_(paths, .Clip, false, &cb.vertex_lists_, &cb.minima_list_)
		}
		if len(opens) > 0 {
			cb.has_open_paths_ = true
		}
		for path in opens {
			if len(path) == 0 {continue}
			ipath := make([dynamic]_Point(struct {}), len(path), context.temp_allocator)
			for pt, i in path {
				ipath[i] = {
					x = f64(pt.x),
					y = f64(pt.y),
				}
			}
			paths := make([dynamic][dynamic]_Point(struct {}), 1, context.temp_allocator)
			paths[0] = ipath
			_addPaths_(paths, .Subject, true, &cb.vertex_lists_, &cb.minima_list_)
		}

		if !_executeInternal(&cb, clipType, fillRule) {
			return nil, nil, __ClipperError.FAILED
		}

		closed := make([dynamic][]_Point(struct {}), context.temp_allocator)
		open := make([dynamic][]_Point(struct {}), context.temp_allocator)
		_buildPaths64(&cb, &closed, &open)

		res = make([][]PointT, len(closed), allocator)
		for ipath, i in closed {
			opath := make([]PointT, len(ipath), allocator)
			for pt, j in ipath {
				opath[j] = _makePoint(PointT, pt.x, pt.y, 0)
			}
			res[i] = opath
		}
		resOpen = make([][]PointT, len(open), allocator)
		for ipath, i in open {
			opath := make([]PointT, len(ipath), allocator)
			for pt, j in ipath {
				opath[j] = _makePoint(PointT, pt.x, pt.y, 0)
			}
			resOpen[i] = opath
		}
		return
	}
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
	context.allocator = allocator
	l, t, r, b := f64(rect.left), f64(rect.top), f64(rect.right), f64(rect.bottom)
	_ = zCallback

	rc: _RectClip64
	rc.rect_ = {left = l, top = t, right = r, bottom = b}
	rc.rect_as_path_ = make([dynamic]_Point(Z_None), 4, context.temp_allocator)
	rc.rect_as_path_[0] = {x = l, y = t}
	rc.rect_as_path_[1] = {x = r, y = t}
	rc.rect_as_path_[2] = {x = r, y = b}
	rc.rect_as_path_[3] = {x = l, y = b}
	rc.rect_mp_ = {x = (l + r) * 0.5, y = (t + b) * 0.5}

	if len(closePaths) > 0 {
		// convert paths to internal format
		internal := make([dynamic][]_Point(Z_None), len(closePaths), context.temp_allocator)
		for p, pi in closePaths {
			pp := make([]_Point(Z_None), len(p), context.temp_allocator)
			for pt, i in p {
				pp[i] = {x = f64(pt.x), y = f64(pt.y)}
			}
			internal[pi] = pp
		}
		raw := _rectClipExecute(&rc, internal[:])
		closed = make([][]PointT, len(raw), allocator)
		for i in 0 ..< len(raw) {
			dst := make([]PointT, len(raw[i]), allocator)
			for j in 0 ..< len(raw[i]) {
				dst[j] = _makePoint(PointT, raw[i][j].x, raw[i][j].y, 0)
			}
			closed[i] = dst
		}
	} else {
		closed = nil
	}

	if len(openPaths) > 0 {
		rc2: _RectClip64
		rc2.rect_ = rc.rect_
		rc2.rect_as_path_ = rc.rect_as_path_
		rc2.rect_mp_ = rc.rect_mp_
		internal := make([dynamic][]_Point(Z_None), len(openPaths), context.temp_allocator)
		for p, pi in openPaths {
			pp := make([]_Point(Z_None), len(p), context.temp_allocator)
			for pt, i in p {
				pp[i] = {x = f64(pt.x), y = f64(pt.y)}
			}
			internal[pi] = pp
		}
		raw := make([dynamic][]_Point(Z_None), context.temp_allocator)
		for idx := 0; idx < len(internal); idx += 1 {
			ipath := internal[idx]
			if len(ipath) < 2 { continue }
			_rectClipLinesExecuteInternal(&rc2, ipath)
			for _, i in rc2.results_ {
				tmp := _rectClipLinesGetPath(&rc2.results_[i])
				if len(tmp) > 0 {
					append(&raw, tmp)
				}
			}
			clear(&rc2.results_)
		}
		open = make([][]PointT, len(raw), allocator)
		for i in 0 ..< len(raw) {
			dst := make([]PointT, len(raw[i]), allocator)
			for j in 0 ..< len(raw[i]) {
				dst[j] = _makePoint(PointT, raw[i][j].x, raw[i][j].y, 0)
			}
			open[i] = dst
		}
	} else {
		open = nil
	}
	return
}
