package geometry

import "./clipper"
import "./linalg_ex"
import "./triangulation"
import "base:intrinsics"
import "base:runtime"
import "core:fmt"
import "core:math"
import "core:math/linalg"
import "core:mem"

import "shared:utils_private"


ShapeVertexFlag :: enum u8 {
	LINE,
	CUBIC,
	QUAD,
}

ShapeVertex2d :: struct #align (16) {
	pos:   linalg.Vector2f32,
	uvw:   linalg.Vector4f32, //마지막 w 는 flag
	color: linalg.Vector4f32,
}

RawShape :: struct {
	vertices: []ShapeVertex2d,
	indices:  []u32,
	rect:     linalg_ex.Rectf32,
}

CurveType :: enum u8 {
	Line,
	Unknown,
	Serpentine,
	Loop,
	// LoopReverse,
	Cusp,
	Quadratic,
}

__ShapeError :: enum {
	IsPointNotLine,
	EmptyPolygon,
	EmptyColor,
	Length_Mismatch,
	Empty_Input,
	First_Point_Is_Curve,
	Consecutive_Anchor_Missing_Control,
	Too_Many_Consecutive_Curves,
	OverFlow,
}

ShapeError :: union #shared_nil {
	__ShapeError,
	triangulation.__TrianguateError,
	clipper.__ClipperError,
	runtime.Allocator_Error,
}

ShapeNode :: struct {
	pts:         [][]linalg.Vector2f32,
	isCurves:    [][]bool,
	color:       linalg.Vector4f32,
	strokeColor: linalg.Vector4f32,
	thickness:   f32,
	isClosed:    bool,
	clipRect:    linalg_ex.Rectf32, // TODO: clipper 안정화 후 shapesComputePolygon에서 처리 활성화
}

Shapes :: struct {
	nodes:    []ShapeNode,
	clipRect: linalg_ex.Rectf32,
}

@(private)
FindParentContour :: proc(idx: int, ptsIn: [][]linalg.Vector2f32) -> int {
	if idx < 0 || idx >= len(ptsIn) do return -1
	if len(ptsIn[idx]) < 3 do return -1

	p := ptsIn[idx][0]

	parentIdx := -1
	parentArea := f32(0)

	for j in 0 ..< len(ptsIn) {
		if j == idx do continue

		other := ptsIn[j]
		if len(other) < 3 do continue

		if linalg_ex.PointInPolygon(p, other) == .Outside {
			continue
		}

		// idx를 포함하는 contour 중 가장 작은 면적의 contour를 parent로 선택
		area := abs(linalg_ex.PolygonSignedArea(other))

		if parentIdx == -1 || area < parentArea {
			parentIdx = j
			parentArea = area
		}
	}

	return parentIdx
}

IsHoleContour :: proc(idx: int, ptsIn: [][]linalg.Vector2f32) -> bool {
	if idx < 0 || idx >= len(ptsIn) do return false
	if len(ptsIn[idx]) < 3 do return false

	parent := FindParentContour(idx, ptsIn)
	if parent == -1 {
		// parent가 없으면 최외곽 contour
		return false
	}

	// parent가 outer면 나는 hole
	// parent가 hole이면 나는 outer
	return !IsHoleContour(parent, ptsIn)
}

ReverseShapeCloseCurve :: proc(
	pts: []linalg.Vector2f32,
	isCurves: []bool,
	allocator := context.allocator,
) -> (
	outPts: []linalg.Vector2f32,
	outIsCurves: []bool,
	err: ShapeError,
) {
	when size_of(int) > 4 { 	//only 64bits can overflow len(int) than max(u32)
		if len(pts) > int(max(u32)) || len(isCurves) > int(max(u32)) {
			return nil, nil, .OverFlow
		}
	}


	n := u32(len(pts))

	if n != u32(len(isCurves)) {
		return nil, nil, .Length_Mismatch
	}
	if n == 0 {
		return nil, nil, .Empty_Input
	}
	if isCurves[0] {
		return nil, nil, .First_Point_Is_Curve
	}

	anchorIndices := make([dynamic]u32, context.temp_allocator) or_return

	for i in 0 ..< n {
		if !isCurves[i] {
			non_zero_append(&anchorIndices, i) or_return
		}
	}

	if len(anchorIndices) == 0 {
		return nil, nil, .First_Point_Is_Curve
	}

	// 각 anchor 사이 control 개수 검사
	for a := 0; a < len(anchorIndices); a += 1 {
		start := anchorIndices[a]
		next := anchorIndices[(a + 1) % len(anchorIndices)]

		curveCount := 0
		i := (start + 1) % n
		for i != next {
			if !isCurves[i] {
				return nil, nil, .Consecutive_Anchor_Missing_Control
			}
			curveCount += 1
			i = (i + 1) % n
		}

		if curveCount == 0 {
			return nil, nil, .Consecutive_Anchor_Missing_Control
		}
		if curveCount > 2 {
			return nil, nil, .Too_Many_Consecutive_Curves
		}
	}

	outPts = utils_private.makeNonZeroedSlice([]linalg.Vector2f32, n, allocator) or_return
	defer if err != nil do delete(outPts, allocator)

	outIsCurves = utils_private.makeNonZeroedSlice([]bool, n, allocator) or_return
	defer if err != nil do delete(outIsCurves, allocator)

	outPts[0] = pts[0]
	outIsCurves[0] = false
	outI: u32 = 1

	m := len(anchorIndices)

	// reverse된 순서대로 모든 세그먼트를 처리
	// step=0: last -> first
	// step=1: second_last -> last
	// ...
	// step=m-1: first -> second
	for step := 0; step < m; step += 1 {
		endAnchorPos := (m - step) % m
		startAnchorPos := (endAnchorPos - 1 + m) % m

		a0 := anchorIndices[startAnchorPos]
		a1 := anchorIndices[endAnchorPos]

		controls := [2]u32{}
		curveCount := 0

		i := (a0 + 1) % n
		for i != a1 {
			controls[curveCount] = i
			curveCount += 1
			i = (i + 1) % n
		}

		switch curveCount {
		case 1:
			outPts[outI] = pts[controls[0]]
			outIsCurves[outI] = true
			outI += 1

		case 2:
			outPts[outI] = pts[controls[1]]
			outIsCurves[outI] = true
			outI += 1

			outPts[outI] = pts[controls[0]]
			outIsCurves[outI] = true
			outI += 1

		case:
			return nil, nil, .Too_Many_Consecutive_Curves
		}

		// 마지막 세그먼트는 first anchor로 닫히므로
		// anchor 0은 다시 쓰지 않음
		if a0 != 0 {
			outPts[outI] = pts[a0]
			outIsCurves[outI] = false
			outI += 1
		}
	}

	if outI != n {
		return nil, nil, .EmptyPolygon
	}

	return outPts, outIsCurves, nil
}

// Same layout as curve_struct_fixed for floating-point coordinates (f16 / f32 / f64).
CurveStructFloat :: struct($F: typeid) where intrinsics.type_is_float(F) {
	start:        [2]F,
	ctl0:         [2]F,
	ctl1:         [2]F,
	end:          [2]F,
	type:         CurveType,
	curveReverse: bool,
}

rawShapeFree :: proc(self: RawShape, allocator := context.allocator) {
	delete(self.vertices, allocator)
	delete(self.indices, allocator)
}

rawShapeComputeRect :: proc "contextless" (self: RawShape) -> linalg_ex.Rectf32 {
	if len(self.vertices) == 0 do return {}
	left := self.vertices[0].pos.x
	right := self.vertices[0].pos.x
	bottom := self.vertices[0].pos.y
	top := self.vertices[0].pos.y
	for v in self.vertices[1:] {
		left = min(left, v.pos.x)
		right = max(right, v.pos.x)
		bottom = min(bottom, v.pos.y)
		top = max(top, v.pos.y)
	}
	return linalg_ex.RectInit(left, right, top, bottom)
}

rawShapeUpdateRect :: proc(self: ^RawShape) {
	if self == nil do return
	self.rect = rawShapeComputeRect(self^)
}

rawShapeClone :: proc(
	self: ^RawShape,
	allocator := context.allocator,
) -> (
	res: ^RawShape = nil,
	err: runtime.Allocator_Error,
) #optional_allocator_error {
	res = new(RawShape, allocator) or_return
	defer if err != nil {
		free(res, allocator)
		res = nil
	}

	res.vertices = utils_private.makeNonZeroedSlice(
		[]ShapeVertex2d,
		len(self.vertices),
		allocator,
	) or_return
	defer if err != nil do delete(res.vertices, allocator)

	res.indices = utils_private.makeNonZeroedSlice([]u32, len(self.indices), allocator) or_return

	intrinsics.mem_copy_non_overlapping(
		&res.vertices[0],
		&self.vertices[0],
		len(self.vertices) * size_of(ShapeVertex2d),
	)
	intrinsics.mem_copy_non_overlapping(
		&res.indices[0],
		&self.indices[0],
		len(self.indices) * size_of(u32),
	)
	res.rect = self.rect
	return
}

GetCubicCurveType :: proc "contextless" (
	start: [2]$T,
	control0: [2]T,
	control1: [2]T,
	end: [2]T,
) -> (
	type: CurveType = .Unknown,
	d0: T,
	d1: T,
	d2: T,
	err: ShapeError = nil,
) where intrinsics.type_is_float(T) {
	if start == control0 && control0 == control1 && control1 == end {
		err = .IsPointNotLine
		return
	}
	start: [2]f64 = [2]f64{f64(start.x), f64(start.y)}
	control0: [2]f64 = [2]f64{f64(control0.x), f64(control0.y)}
	control1: [2]f64 = [2]f64{f64(control1.x), f64(control1.y)}
	end: [2]f64 = [2]f64{f64(end.x), f64(end.y)}

	cross1 := [3]f64 {
		end.y - control1.y,
		control1.x - end.x,
		end.x * control1.y - end.y * control1.x,
	}
	cross2 := [3]f64{start.y - end.y, end.x - start.x, start.x * end.y - start.y * end.x}
	cross3 := [3]f64 {
		control0.y - start.y,
		start.x - control0.x,
		control0.x * start.y - control0.y * start.x,
	}

	a1 := start.x * cross1.x + start.y * cross1.y + cross1.z //9
	a2 := control0.x * cross2.x + control0.y * cross2.y + cross2.z //7
	a3 := control1.x * cross3.x + control1.y * cross3.y + cross3.z //7

	d0_: f64 = a1 - 2.0 * a2 + 3.0 * a3 //27
	d1_: f64 = -a2 + 3.0 * a3 //16
	d2_: f64 = 3.0 * a3 //8

	D := 3.0 * d1_ * d1_ - 4.0 * d2_ * d0_ //33 + 36 + 1 = 70
	discr := d0_ * d0_ * D //27 + 27 + 70 = 124

	EP: f64 = linalg_ex.epsilon(f64)
	d0 = T(d0_)
	d1 = T(d1_)
	d2 = T(d2_)
	if discr >= -EP && discr <= EP {
		if d0_ == 0.0 && d1_ == 0.0 {
			if d2_ == 0.0 {
				type = .Line
				return
			}
			type = .Quadratic
			return
		}
		type = .Cusp
		return
	}
	if discr > 0.0 {
		type = .Serpentine
		return
	}
	type = .Loop
	return
}

@(private = "file")
_ShapesComputeLine :: proc(
	vertList: ^[dynamic]ShapeVertex2d,
	indList: ^[dynamic]u32,
	color: linalg.Vector4f32,
	pts: ^CurveStructFloat(f32), //inout
) -> ShapeError {
	curveType := pts.type
	err: ShapeError = nil
	//loop_reverse := false
	//if curveType == .LoopReverse do loop_reverse = true

	pts.curveReverse = false
	reverse := false
	d0, d1, d2: f32
	if curveType != .Line && curveType != .Quadratic {
		curveType, d0, d1, d2 = GetCubicCurveType(pts.start, pts.ctl0, pts.ctl1, pts.end) or_return
		//pts.type = curveType
	} else if curveType == .Quadratic {
		vlen: u32 = u32(len(vertList))
		quadSign := f32(1.0)
		if linalg_ex.GetPolygonOrientation([][2]f32{pts.start, pts.ctl0, pts.end}) ==
		   .CounterClockwise {
			quadSign = -1.0
			pts.curveReverse = true
		}
		non_zero_append(
			vertList,
			ShapeVertex2d {
				uvw = {0.0, 0.0, quadSign, f32(ShapeVertexFlag.QUAD)},
				pos = linalg.Vector2f32{pts.start.x, pts.start.y},
				color = color,
			},
		)
		non_zero_append(
			vertList,
			ShapeVertex2d {
				uvw = {0.5, 0.0, quadSign, f32(ShapeVertexFlag.QUAD)},
				pos = linalg.Vector2f32{pts.ctl0.x, pts.ctl0.y},
				color = color,
			},
		)
		non_zero_append(
			vertList,
			ShapeVertex2d {
				uvw = {1.0, 1.0, quadSign, f32(ShapeVertexFlag.QUAD)},
				pos = linalg.Vector2f32{pts.end.x, pts.end.y},
				color = color,
			},
		)
		non_zero_append(indList, vlen, vlen + 1, vlen + 2)
		return nil
	}
	F: [4][3]f32

	reverseOrientation :: #force_inline proc "contextless" (F: [4][3]f32) -> [4][3]f32 {
		return [4][3]f32 {
			{-F[0][0], -F[0][1], F[0][2]},
			{-F[1][0], -F[1][1], F[1][2]},
			{-F[2][0], -F[2][1], F[2][2]},
			{-F[3][0], -F[3][1], F[3][2]},
		}
	}

	#partial switch curveType {
	case .Serpentine:
		t1 := math.sqrt_f32(9 * d1 * d1 - 12 * d0 * d2)
		ls := 3 * d1 - t1
		lt := 6 * d0

		ms := 3 * d1 + t1
		mt := lt
		ltMinusLs := lt - ls
		mtMinusMs := mt - ms

		F = {
			{ls * ms, ls * ls * ls, ms * ms * ms},
			{
				(1.0 / 3.0) * (3.0 * ls * ms - ls * mt - lt * ms),
				ls * ls * (ls - lt),
				ms * ms * (ms - mt),
			},
			{
				(1.0 / 3.0) * (lt * (mt - 2.0 * ms) + ls * (3.0 * ms - 2.0 * mt)),
				ltMinusLs * ltMinusLs * ls,
				mtMinusMs * mtMinusMs * ms,
			},
			{
				ltMinusLs * mtMinusMs,
				-(ltMinusLs * ltMinusLs * ltMinusLs),
				-(mtMinusMs * mtMinusMs * mtMinusMs),
			},
		}

		if d0 < 0 do reverse = true
	case .Loop:
		t1: f32 = math.sqrt_f32(4 * d0 * d2 - 3 * d1 * d1)
		ls := d1 - t1
		lt := 2 * d0
		ms := d1 + t1
		mt := lt

		ltMinusLs := lt - ls
		mtMinusMs := mt - ms

		F = {
			{ls * ms, ls * ls * ms, ls * ms * ms},
			{
				(1.0 / 3.0) * (-ls * mt - lt * ms + 3.0 * ls * ms),
				-(1.0 / 3.0) * ls * (ls * (mt - 3.0 * ms) + 2.0 * lt * ms),
				-(1.0 / 3.0) * ms * (ls * (2.0 * mt - 3.0 * ms) + lt * ms),
			},
			{
				(1.0 / 3.0) * (lt * (mt - 2.0 * ms) + ls * (3.0 * ms - 2.0 * mt)),
				(1.0 / 3.0) * ltMinusLs * (ls * (2.0 * mt - 3.0 * ms) + lt * ms),
				(1.0 / 3.0) * mtMinusMs * (ls * (mt - 3.0 * ms) + 2.0 * lt * ms),
			},
			{
				ltMinusLs * mtMinusMs,
				-(ltMinusLs * ltMinusLs) * mtMinusMs,
				-ltMinusLs * mtMinusMs * mtMinusMs,
			},
		}

		reverse = (d0 > 0 && F[1][0] < 0.0) || (d0 < 0 && F[1][0] > 0.0)
	case .Cusp:
		ls := d2
		lt := 3.0 * d1
		lsMinusLt := ls - lt
		F = {
			{ls, ls * ls * ls, 1.0},
			{(ls - (1.0 / 3.0) * lt), ls * ls * lsMinusLt, 1.0},
			{ls - (2.0 / 3.0) * lt, lsMinusLt * lsMinusLt * ls, 1.0},
			{lsMinusLt, lsMinusLt * lsMinusLt * lsMinusLt, 1.0},
		}
	case .Quadratic:
		F = {
			{0.0, 0.0, 0.0},
			{1.0 / 3.0, 0.0, 1.0 / 3.0},
			{2.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0},
			{1.0, 1.0, 1.0},
		}
		if d2 < 0 do reverse = true
	case .Line:
		return nil
	//reverse = true
	// case .Unknown:
	//     unreachable()
	}

	//if loop_reverse do reverse = !reverse
	pts.curveReverse = reverse
	if reverse {
		F = reverseOrientation(F)
	}

	appendLine :: proc(
		vertList: ^[dynamic]ShapeVertex2d,
		indList: ^[dynamic]u32,
		color: linalg.Vector4f32,
		pts: CurveStructFloat(f32),
		F: [4][3]f32,
	) -> ShapeError {
		start: u32 = u32(len(vertList))
		non_zero_append(
			vertList,
			ShapeVertex2d {
				uvw = {F[0][0], F[0][1], F[0][2], f32(ShapeVertexFlag.CUBIC)},
				color = color,
			},
		) or_return
		non_zero_append(
			vertList,
			ShapeVertex2d {
				uvw = {F[1][0], F[1][1], F[1][2], f32(ShapeVertexFlag.CUBIC)},
				color = color,
			},
		) or_return
		non_zero_append(
			vertList,
			ShapeVertex2d {
				uvw = {F[2][0], F[2][1], F[2][2], f32(ShapeVertexFlag.CUBIC)},
				color = color,
			},
		) or_return
		non_zero_append(
			vertList,
			ShapeVertex2d {
				uvw = {F[3][0], F[3][1], F[3][2], f32(ShapeVertexFlag.CUBIC)},
				color = color,
			},
		) or_return

		vertList[start].pos = linalg.Vector2f32{pts.start.x, pts.start.y}
		vertList[start + 1].pos = linalg.Vector2f32{pts.ctl0.x, pts.ctl0.y}
		vertList[start + 2].pos = linalg.Vector2f32{pts.ctl1.x, pts.ctl1.y}
		vertList[start + 3].pos = linalg.Vector2f32{pts.end.x, pts.end.y}
		//triangulate
		vts: [4]linalg.Vector2f32 = {
			vertList[start].pos,
			vertList[start + 1].pos,
			vertList[start + 2].pos,
			vertList[start + 3].pos,
		}

		for i: u32 = 0; i < 4; i += 1 {
			for j: u32 = i + 1; j < 4; j += 1 {
				if vts[i] == vts[j] {
					indices: [3]u32 = {start, start, start}
					idx: u32 = 0
					for k: u32 = 0; k < 4; k += 1 {
						if k != j {
							indices[idx] += k
							idx += 1
						}
					}
					non_zero_append(indList, ..indices[:]) or_return
					return nil
				}
			}
		}

		for i: u32 = 0; i < 4; i += 1 {
			indices: [3]u32 = {start, start, start}
			idx: u32 = 0
			for j: u32 = 0; j < 4; j += 1 {
				if j != i {
					indices[idx] += j
					idx += 1
				}
			}
			if linalg_ex.PointInTriangle(
				vts[i],
				vts[indices[0] - start],
				vts[indices[1] - start],
				vts[indices[2] - start],
			) {
				for k: u32 = 0; k < 3; k += 1 {
					non_zero_append(indList, indices[k]) or_return
					non_zero_append(indList, indices[(k + 1) % 3]) or_return
					non_zero_append(indList, start + i) or_return
				}
				return nil
			}
		}

		b := linalg_ex.LinesIntersect3(vts[0], vts[2], vts[1], vts[3])
		if b == .intersect {
			if linalg.vector_length2(vts[2] - vts[0]) < linalg.vector_length2(vts[3] - vts[1]) {
				non_zero_append(
					indList,
					start,
					start + 1,
					start + 2,
					start,
					start + 2,
					start + 3,
				) or_return
			} else {
				non_zero_append(
					indList,
					start,
					start + 1,
					start + 3,
					start + 1,
					start + 2,
					start + 3,
				) or_return
			}
			return nil
		}
		b = linalg_ex.LinesIntersect3(vts[0], vts[3], vts[1], vts[2])
		if b == .intersect {
			if linalg.vector_length2(vts[3] - vts[0]) < linalg.vector_length2(vts[2] - vts[1]) {
				non_zero_append(indList, start, start + 1, start + 3, start, start + 3, start + 2)
			} else {
				non_zero_append(
					indList,
					start,
					start + 1,
					start + 2,
					start + 2,
					start + 1,
					start + 3,
				) or_return
			}
			return nil
		}
		if linalg.vector_length2(vts[1] - vts[0]) < linalg.vector_length2(vts[3] - vts[2]) {
			non_zero_append(
				indList,
				start,
				start + 2,
				start + 1,
				start,
				start + 1,
				start + 3,
			) or_return
		} else {
			non_zero_append(
				indList,
				start,
				start + 2,
				start + 3,
				start + 3,
				start + 2,
				start + 1,
			) or_return
		}
		return nil
	}
	appendLine(vertList, indList, color, pts^, F) or_return

	return nil
}

@(private = "file")
SubdivCurveAndInjectAt :: proc(
	curvesN: ^[dynamic]CurveStructFloat(f32),
	insertIdx: int,
	cur: ^CurveStructFloat(f32),
	src: CurveStructFloat(f32),
	t: f32,
) -> (
	mid: [2]f32,
	c0: [2]f32,
	c1: [2]f32,
	err: ShapeError = nil,
) {
	if src.type == .Quadratic {
		p0, p1_, p2_ := linalg_ex.SubdivQuadraticBezier([3][2]f32{src.start, src.ctl0, src.end}, t)
		cur.ctl0, cur.end = p0, p1_
		mid, c0 = p1_, p2_
		utils_private.nonZeroInjectAtElem(
			curvesN,
			insertIdx,
			CurveStructFloat(f32){start = p1_, ctl0 = p2_, end = src.end, type = .Quadratic},
		) or_return
	} else {
		p0, p1_, p2_, p3_, p4_ := linalg_ex.SubdivCubicBezier(
			[4][2]f32{src.start, src.ctl0, src.ctl1, src.end},
			t,
		)
		cur.ctl0, cur.ctl1, cur.end = p0, p1_, p2_
		mid, c0, c1 = p2_, p3_, p4_
		utils_private.nonZeroInjectAtElem(
			curvesN,
			insertIdx,
			CurveStructFloat(f32) {
				start = p2_,
				ctl0 = p3_,
				ctl1 = p4_,
				end = src.end,
				type = .Unknown,
			},
		) or_return
	}
	return
}

@(private = "file")
SubdivCurveAt :: proc "contextless" (
	src: CurveStructFloat(f32),
	t: f32,
) -> (
	left, right: CurveStructFloat(f32),
) {
	if src.type == .Quadratic {
		p0, p1_, p2_ := linalg_ex.SubdivQuadraticBezier([3][2]f32{src.start, src.ctl0, src.end}, t)
		left = CurveStructFloat(f32) {
			start = src.start,
			ctl0  = p0,
			end   = p1_,
			type  = .Quadratic,
		}
		right = CurveStructFloat(f32) {
			start = p1_,
			ctl0  = p2_,
			end   = src.end,
			type  = .Quadratic,
		}
	} else {
		p0, p1_, p2_, p3_, p4_ := linalg_ex.SubdivCubicBezier(
			[4][2]f32{src.start, src.ctl0, src.ctl1, src.end},
			t,
		)
		left = CurveStructFloat(f32) {
			start = src.start,
			ctl0  = p0,
			ctl1  = p1_,
			end   = p2_,
			type  = .Unknown,
		}
		right = CurveStructFloat(f32) {
			start = p2_,
			ctl0  = p3_,
			ctl1  = p4_,
			end   = src.end,
			type  = .Unknown,
		}
	}
	return
}

@(private = "file")
CurveChordOverlaps :: proc(src: CurveStructFloat(f32), cur: CurveStructFloat(f32)) -> bool {
	if src.type == .Line && cur.type == .Line do return false

	if src.type == .Line || cur.type == .Line {
		// curve-vs-line: use curve chord (start-end) vs line (start-end)
		k, _ := linalg_ex.LinesIntersect2(src.start, src.end, cur.start, cur.end, true)
		return k == .intersect
	}

	k1, _ := linalg_ex.LinesIntersect2(src.start, src.end, cur.start, cur.ctl0, true)
	if k1 == .intersect do return true

	k2, _ := linalg_ex.LinesIntersect2(
		src.start,
		src.end,
		cur.type == .Quadratic ? cur.ctl0 : cur.ctl1,
		cur.end,
		true,
	)
	return k2 == .intersect
}

@(private = "file")
CurveChordOverlapsAny :: proc(
	srcs: []CurveStructFloat(f32),
	curs: []CurveStructFloat(f32),
) -> bool {
	for src in srcs {
		for cur in curs {
			if CurveChordOverlaps(src, cur) do return true
		}
	}
	return false
}

@(private = "file")
SubdivCurveSegmentsAtHalf :: proc "contextless" (
	srcs: []CurveStructFloat(f32),
	dst: []CurveStructFloat(f32),
) {
	// caller guarantees len(dst) == len(srcs) * 2
	for src, idx in srcs {
		a, b := SubdivCurveAt(src, 0.5)
		dst[2 * idx] = a
		dst[2 * idx + 1] = b
	}
	return
}

@(private = "file")
InjectSubdivCurveSegments :: proc(
	curvesN: ^[dynamic]CurveStructFloat(f32),
	splitFlagsN: ^[dynamic]bool,
	atIdx: int,
	segs: []CurveStructFloat(f32),
) -> (
	added: int,
	err: ShapeError = nil,
) {
	if len(segs) == 0 do return

	curvesN[atIdx] = segs[0]
	splitFlagsN[atIdx] = true
	if len(segs) > 1 {
		utils_private.nonZeroInjectAtElems(curvesN, atIdx + 1, ..segs[1:]) or_return
		for k := 1; k < len(segs); k += 1 {
			utils_private.nonZeroInjectAtElem(splitFlagsN, atIdx + k, true) or_return
		}
	}
	added = len(segs) - 1
	return
}

@(private = "file")
ProcessCurveOverlapPair :: proc(
	curvesA: ^[dynamic]CurveStructFloat(f32),
	splitFlagsA: ^[dynamic]bool,
	i: int,
	curvesB: ^[dynamic]CurveStructFloat(f32),
	splitFlagsB: ^[dynamic]bool,
	j: int,
	sameContour: bool,
) -> (
	curAdded: int,
	srcAdded: int,
	overlapped: bool,
	err: ShapeError = nil,
) {
	src := curvesB[j]
	cur := curvesA[i]

	srcIsLine := src.type == .Line
	curIsLine := cur.type == .Line
	if srcIsLine && curIsLine do return
	if !CurveChordOverlaps(src, cur) do return
	overlapped = true

	// curve-vs-line: split only the curve side.
	if srcIsLine || curIsLine {
		if srcIsLine {
			curHalf0, curHalf1 := SubdivCurveAt(cur, 0.5)
			curSegs2 := [2]CurveStructFloat(f32){curHalf0, curHalf1}
			curSegs4: [4]CurveStructFloat(f32)
			curSegs8: [8]CurveStructFloat(f32)
			curSegs: []CurveStructFloat(f32) = curSegs2[:]
			lineProbe := [1]CurveStructFloat(f32){src}

			if CurveChordOverlapsAny(lineProbe[:], curSegs) {
				SubdivCurveSegmentsAtHalf(curSegs, curSegs4[:])
				curSegs = curSegs4[:]
				if CurveChordOverlapsAny(lineProbe[:], curSegs) {
					SubdivCurveSegmentsAtHalf(curSegs, curSegs8[:])
					curSegs = curSegs8[:]
				}
			}
			curAdded = InjectSubdivCurveSegments(curvesA, splitFlagsA, i, curSegs) or_return
		} else {
			srcHalf0, srcHalf1 := SubdivCurveAt(src, 0.5)
			srcSegs2 := [2]CurveStructFloat(f32){srcHalf0, srcHalf1}
			srcSegs4: [4]CurveStructFloat(f32)
			srcSegs8: [8]CurveStructFloat(f32)
			srcSegs: []CurveStructFloat(f32) = srcSegs2[:]
			lineProbe := [1]CurveStructFloat(f32){cur}

			if CurveChordOverlapsAny(srcSegs, lineProbe[:]) {
				SubdivCurveSegmentsAtHalf(srcSegs, srcSegs4[:])
				srcSegs = srcSegs4[:]
				if CurveChordOverlapsAny(srcSegs, lineProbe[:]) {
					SubdivCurveSegmentsAtHalf(srcSegs, srcSegs8[:])
					srcSegs = srcSegs8[:]
				}
			}

			if sameContour {
				srcAdded = InjectSubdivCurveSegments(curvesA, splitFlagsA, j, srcSegs) or_return
			} else {
				srcAdded = InjectSubdivCurveSegments(curvesB, splitFlagsB, j, srcSegs) or_return
			}
		}
		return
	}

	srcHalf0, srcHalf1 := SubdivCurveAt(src, 0.5)
	curHalf0, curHalf1 := SubdivCurveAt(cur, 0.5)
	srcSegs2 := [2]CurveStructFloat(f32){srcHalf0, srcHalf1}
	curSegs2 := [2]CurveStructFloat(f32){curHalf0, curHalf1}
	curSegs1 := [1]CurveStructFloat(f32){cur}
	srcSegs4: [4]CurveStructFloat(f32)
	srcSegs8: [8]CurveStructFloat(f32)
	curSegs4: [4]CurveStructFloat(f32)
	curSegs8: [8]CurveStructFloat(f32)
	srcSegs: []CurveStructFloat(f32) = srcSegs2[:]
	curSegs: []CurveStructFloat(f32) = curSegs1[:]

	// refine one side at a time:
	// src(2) -> cur(2) -> src(4) -> cur(4) -> src(8) -> cur(8)
	if CurveChordOverlapsAny(srcSegs, curSegs) {
		curSegs = curSegs2[:]

		if CurveChordOverlapsAny(srcSegs, curSegs) {
			SubdivCurveSegmentsAtHalf(srcSegs, srcSegs4[:])
			srcSegs = srcSegs4[:]

			if CurveChordOverlapsAny(srcSegs, curSegs) {
				SubdivCurveSegmentsAtHalf(curSegs, curSegs4[:])
				curSegs = curSegs4[:]

				if CurveChordOverlapsAny(srcSegs, curSegs) {
					SubdivCurveSegmentsAtHalf(srcSegs, srcSegs8[:])
					srcSegs = srcSegs8[:]

					if CurveChordOverlapsAny(srcSegs, curSegs) {
						SubdivCurveSegmentsAtHalf(curSegs, curSegs8[:])
						curSegs = curSegs8[:]
					}
				}
			}
		}
	}

	if sameContour {
		if len(srcSegs) > 1 && len(curSegs) > 1 {
			if j > i {
				srcAdded = InjectSubdivCurveSegments(curvesA, splitFlagsA, j, srcSegs) or_return
				curAdded = InjectSubdivCurveSegments(curvesA, splitFlagsA, i, curSegs) or_return
			} else {
				curAdded = InjectSubdivCurveSegments(curvesA, splitFlagsA, i, curSegs) or_return
				srcAdded = InjectSubdivCurveSegments(curvesA, splitFlagsA, j, srcSegs) or_return
			}
		} else if len(srcSegs) > 1 {
			srcAdded = InjectSubdivCurveSegments(curvesA, splitFlagsA, j, srcSegs) or_return
		} else if len(curSegs) > 1 {
			curAdded = InjectSubdivCurveSegments(curvesA, splitFlagsA, i, curSegs) or_return
		}
	} else {
		srcAdded = InjectSubdivCurveSegments(curvesB, splitFlagsB, j, srcSegs) or_return
		if len(curSegs) > 1 {
			curAdded = InjectSubdivCurveSegments(curvesA, splitFlagsA, i, curSegs) or_return
		}
	}
	return
}

shapesComputePolygon :: proc(
	poly: Shapes,
	allocator := context.allocator,
) -> (
	res: RawShape,
	err: ShapeError = nil,
) {
	vertList: [dynamic]ShapeVertex2d = make([dynamic]ShapeVertex2d, context.temp_allocator)
	indList: [dynamic]u32 = make([dynamic]u32, context.temp_allocator)

	shapesComputePolygonIn :: proc(
		vertList: ^[dynamic]ShapeVertex2d,
		indList: ^[dynamic]u32,
		poly: Shapes,
	) -> (
		err: ShapeError = nil,
	) {
		nonCurves := make([dynamic][][2]f32, context.temp_allocator)
		nonCurves2: [dynamic][dynamic][2]f32 = make(
			[dynamic][dynamic][2]f32,
			context.temp_allocator,
		)
		curves2: [dynamic][dynamic]CurveStructFloat(f32) = make(
			[dynamic][dynamic]CurveStructFloat(f32),
			context.temp_allocator,
		)
		overlapSkip2: [dynamic][dynamic]bool = make([dynamic][dynamic]bool, context.temp_allocator)
		insertAr: [dynamic; 2][2]f32

		for node, nidx in poly.nodes {
			if node.color.a > 0 {
				// TODO: clipper 안정화 후 주석 해제
				// pts := node.pts
				// isCurves := node.isCurves
				// _isClosed := node.isClosed
				// clipRect := node.clipRect
				// if clipRect.left == 0 && clipRect.right == 0 && clipRect.top == 0 && clipRect.bottom == 0 {
				// 	clipRect = poly.clipRect
				// }
				// if clipRect.left != 0 || clipRect.right != 0 || clipRect.top != 0 || clipRect.bottom != 0 {
				// 	pathsForClip := make([][][2]f32, len(pts), context.temp_allocator)
				// 	curveFlagsForClip := make([][]bool, len(pts), context.temp_allocator)
				// 	for npii in 0 ..< len(pts) {
				// 		pathsForClip[npii] = transmute([][2]f32)pts[npii]
				// 		if isCurves != nil && npii < len(isCurves) {
				// 			curveFlagsForClip[npii] = isCurves[npii]
				// 		}
				// 	}
				// 	clipped, clippedCurves, clipErr := clipper.RectClip(clipRect, pathsForClip[:], curveFlagsForClip[:], context.temp_allocator)
				// 	if clipErr == nil && len(clipped) > 0 {
				// 		pts = make([][]linalg.Vector2f32, len(clipped), context.temp_allocator)
				// 		isCurves = make([][]bool, len(clippedCurves), context.temp_allocator)
				// 		for i in 0 ..< len(clipped) {
				// 			pts[i] = transmute([]linalg.Vector2f32)clipped[i]
				// 			if len(clippedCurves) > i { isCurves[i] = clippedCurves[i] }
				// 		}
				// 	}
				// 	// 주석 해제 시 아래 nonCurves2 resize를 len(pts)로 변경할 것
				// 	// non_zero_resize_dynamic_array(&nonCurves2, len(pts)) or_return
				// 	// non_zero_resize_dynamic_array(&curves2, len(pts)) or_return
				// 	// non_zero_resize_dynamic_array(&overlapSkip2, len(pts)) or_return
				// 	// 아래 for i, for np 루프에서 node.pts → pts, node.isCurves → isCurves, node.isClosed → _isClosed 로 변경할 것
				// }

				non_zero_resize_dynamic_array(&nonCurves2, len(node.pts)) or_return
				non_zero_resize_dynamic_array(&curves2, len(node.pts)) or_return
				non_zero_resize_dynamic_array(&overlapSkip2, len(node.pts)) or_return

				for i in 0 ..< len(node.pts) {
					if nonCurves2[i] == nil do nonCurves2[i] = make([dynamic][2]f32, context.temp_allocator) or_return
					if curves2[i] == nil do curves2[i] = make([dynamic]CurveStructFloat(f32), context.temp_allocator) or_return
					if overlapSkip2[i] == nil do overlapSkip2[i] = make([dynamic]bool, context.temp_allocator) or_return

					non_zero_resize_dynamic_array(&nonCurves2[i], 0) or_return
					non_zero_resize_dynamic_array(&curves2[i], 0) or_return
					non_zero_resize_dynamic_array(&overlapSkip2[i], 0) or_return
				}

				for np, npi in node.pts {
					curveFlags: []bool = nil
					if node.isCurves != nil && npi < len(node.isCurves) {
						curveFlags = node.isCurves[npi]
					}

					if len(np) == 0 do continue

					last := len(np) - 1
					i := 0
					for {
						if !node.isClosed && i >= last do break

						next := node.isClosed ? (i + 1) % len(np) : i + 1
						if next >= len(np) do break

						if next < len(curveFlags) && curveFlags[next] {
							next2 := node.isClosed ? (next + 1) % len(np) : next + 1
							if next2 >= len(np) do break

							if next2 < len(curveFlags) && curveFlags[next2] {
								next3 := node.isClosed ? (next2 + 1) % len(np) : next2 + 1
								if next3 >= len(np) do break

								non_zero_append(
									&curves2[npi],
									CurveStructFloat(f32) {
										start = np[i],
										ctl0 = np[next],
										ctl1 = np[next2],
										end = np[next3],
										type = .Unknown,
									},
								) or_return
								i = next3
							} else {
								non_zero_append(
									&curves2[npi],
									CurveStructFloat(f32) {
										start = np[i],
										ctl0 = np[next],
										end = np[next2],
										type = .Quadratic,
									},
								) or_return
								i = next2
							}
						} else {
							non_zero_append(
								&curves2[npi],
								CurveStructFloat(f32){start = np[i], end = np[next], type = .Line},
							) or_return
							i = next
						}

						if node.isClosed && i == 0 do break
					}
				}

				// Loop subdivision (float)
				for npi in 0 ..< len(curves2) {
					curvesNpi := &curves2[npi]
					for i := 0; i < len(curvesNpi); i += 1 {
						c := curvesNpi[i]
						if c.type != .Quadratic && c.type != .Line {
							curveType, d0, d1, d2 := GetCubicCurveType(
								c.start,
								c.ctl0,
								c.ctl1,
								c.end,
							) or_return
							if curveType == .Loop {
								disc := 4 * d0 * d2 - 3 * d1 * d1
								t1 := math.sqrt_f32(disc)
								ls := d1 - t1
								lt := 2 * d0
								ms := d1 + t1
								mt := lt

								ql := ls / lt
								qm := ms / mt

								subdivAt: f32
								if 0.0 < ql && ql < 1.0 {
									subdivAt = ql
								} else if 0.0 < qm && qm < 1.0 {
									subdivAt = qm
								} else {
									continue
								}

								SubdivCurveAndInjectAt(
									curvesNpi,
									i + 1,
									&curvesNpi[i],
									c,
									subdivAt,
								) or_return
								i += 1
							}
						}
					}
				}

				for npi in 0 ..< len(curves2) {
					skipNpi := &overlapSkip2[npi]
					resize_dynamic_array(skipNpi, len(curves2[npi])) or_return // false by default
				}

				// Overlap check (float)
				// Scan all contour pairs (both directions). Skip logic handles repeat work.
				for npiA in 0 ..< len(curves2) {
					curvesA := &curves2[npiA]
					skipA := &overlapSkip2[npiA]

					for npiB := 0; npiB < len(curves2); npiB += 1 {
						curvesB := &curves2[npiB]
						skipB := &overlapSkip2[npiB]
						sameContour := npiA == npiB

						for i := 0; i < len(curvesA); i += 1 {
							if skipA[i] do continue

							for j := 0; j < len(curvesB); j += 1 {
								if i >= len(curvesA) do break
								if skipA[i] do break
								if skipB[j] do continue
								if sameContour && i == j do continue

								origI, origJ := i, j
								curAdded, srcAdded, overlapped := ProcessCurveOverlapPair(
									curvesA,
									skipA,
									i,
									curvesB,
									skipB,
									j,
									sameContour,
								) or_return
								if !overlapped do continue

								if sameContour {
									if srcAdded > 0 && origJ < origI do i += srcAdded
									if curAdded > 0 && origI < origJ do j += curAdded
								}

								if curAdded > 0 do i += curAdded
								if srcAdded > 0 do j += srcAdded
								if i >= len(curvesA) do break
							}
						}
					}
				}

				for npi in 0 ..< len(node.pts) {
					nonCurvesNpi := &nonCurves2[npi]
					curvesNpi := curves2[npi][:]

					non_zero_resize_dynamic_array(nonCurvesNpi, 0) or_return
					if len(curvesNpi) == 0 do continue

					non_zero_append(nonCurvesNpi, curvesNpi[0].start) or_return
					for c, i in curvesNpi {
						if node.isClosed &&
						   i == len(curvesNpi) - 1 &&
						   c.end == curvesNpi[0].start {
							continue
						}
						non_zero_append(nonCurvesNpi, c.end) or_return
					}
				}
				for c, i in curves2 {
					for &cc in c {
						_ShapesComputeLine(vertList, indList, node.color, &cc) or_return
					}
				}

				// Insert curve control points into polygon boundaries (float)
				for npi in 0 ..< len(node.pts) {
					nonCurvesNpi := &nonCurves2[npi]
					curvesNpi := curves2[npi]
					polyPts := nonCurvesNpi[:]

					for c in curvesNpi {
						if c.type != .Line {
							non_zero_resize_fixed_capacity_dynamic_array(&insertAr, 0)

							if c.type != .Quadratic {
								if linalg_ex.PointLineLeftOrRight(c.ctl0, c.start, c.end) > 0 {
									append_fixed_capacity_elem(&insertAr, c.ctl0)
								}
								if linalg_ex.PointLineLeftOrRight(c.ctl1, c.start, c.end) > 0 {
									append_fixed_capacity_elem(&insertAr, c.ctl1)
								}
							} else {
								if linalg_ex.PointLineLeftOrRight(c.ctl0, c.start, c.end) > 0 {
									append_fixed_capacity_elem(&insertAr, c.ctl0)
								}
							}

							if len(insertAr) > 0 {
								insertIdx := -1
								for idx := 0; idx < len(nonCurvesNpi); idx += 1 {
									next := idx + 1
									if node.isClosed {
										next %= len(nonCurvesNpi)
									} else if next >= len(nonCurvesNpi) {
										break
									}

									if nonCurvesNpi[idx] == c.start &&
									   nonCurvesNpi[next] == c.end {
										insertIdx = next
										break
									}
								}
								if insertIdx < 0 {
									continue
								}
								utils_private.nonZeroInjectAtElems(
									nonCurvesNpi,
									insertIdx,
									..insertAr[:],
								) or_return
								polyPts = nonCurvesNpi[:]
							}
						}
					}
				}

				non_zero_resize_dynamic_array(&nonCurves, len(nonCurves2)) or_return
				for npi in 0 ..< len(nonCurves2) do nonCurves[npi] = nonCurves2[npi][:]

				indices, triErr := triangulation.TrianguatePolygons(
					nonCurves[:],
					context.temp_allocator,
					u32(len((vertList^))),
				)
				if triErr != nil {
					switch e in triErr {
					case triangulation.__TrianguateError:
						return e
					case runtime.Allocator_Error:
						return e
					}
				}

				for n in nonCurves {
					for nn in n {
						non_zero_append(
							vertList,
							ShapeVertex2d{color = node.color, pos = nn},
						) or_return
					}
				}
				non_zero_append(indList, ..indices) or_return
			}
		}
		return
	}

	shapesComputePolygonIn(&vertList, &indList, poly) or_return

	res.vertices = utils_private.makeNonZeroedSlice(
		[]ShapeVertex2d,
		len(vertList),
		allocator,
	) or_return
	res.indices = utils_private.makeNonZeroedSlice([]u32, len(indList), allocator) or_return
	mem.copy_non_overlapping(
		raw_data(res.vertices),
		raw_data(vertList[:]),
		len(vertList) * size_of(ShapeVertex2d),
	)
	mem.copy_non_overlapping(
		raw_data(res.indices),
		raw_data(indList[:]),
		len(indList) * size_of(u32),
	)
	rawShapeUpdateRect(&res)
	return
}

polyTransformMatrix :: proc "contextless" (inoutPoly: ^Shapes, F: linalg.Matrix4x4f32) {
	for &node in inoutPoly.nodes {
		for pts in node.pts {
			for &pt in pts {
				out := linalg.mul(F, linalg.Vector4f32{pt.x, pt.y, 0.0, 1.0})
				pt = linalg.Vector2f32{out.x, out.y} / out.w
			}
		}
	}
}
