package svg

import ".."
import "base:runtime"
import "core:encoding/xml"
import "core:math"
import "core:math/linalg"
import "core:mem"
import "core:strconv"
import "core:strings"
import pv "shared:clibs/plutovg"
import "shared:geometry/linalg_ex"
import "shared:utils_private"

__SvgError :: enum {
	NOT_INITIALIZED,
	INVALID_NODE,
	UNSUPPORTED_FEATURE,
}

SvgError :: union #shared_nil {
	__SvgError,
	xml.Error,
	runtime.Allocator_Error,
}

SvgParser :: struct {
	arenaAllocator: Maybe(mem.Allocator),
	__arena:         mem.Dynamic_Arena,
	shapes:          geometry.Shapes,
}

@(private)
__PathAttr :: struct {
	d:              Maybe(string),
	fill:           Maybe(string),
	fillOpacity:   Maybe(f32),
	stroke:         Maybe(string),
	strokeOpacity: Maybe(f32),
	strokeWidth:   Maybe(f32),
	transform:      Maybe(string),
}

_EPS: f32 : 0.0001

deinit :: proc(self: ^SvgParser) {
	if self == nil do return
	if self.arenaAllocator != nil {
		mem.dynamic_arena_destroy(&self.__arena)
		self.arenaAllocator = nil
	}
	self.shapes = {}
}

@(private)
_isNoneValue :: proc "contextless" (s: string) -> bool {
	if len(s) != 4 do return false
	return(
		(s[0] == 'n' || s[0] == 'N') &&
		(s[1] == 'o' || s[1] == 'O') &&
		(s[2] == 'n' || s[2] == 'N') &&
		(s[3] == 'e' || s[3] == 'E') \
	)
}


@(private)
_toGeomPoint :: proc "contextless" (p: pv.plutovg_point_t) -> linalg.Vector2f32 {
	return linalg.Vector2f32{p.x, -p.y}
}

@(private)
_parseNumberAttr :: proc(value: string, out: ^Maybe(f32)) -> SvgError {
	v := strings.trim_space(value)
	f, ok := strconv.parse_f32(v)
	if !ok do return .INVALID_NODE
	out^ = f
	return nil
}

@(private)
_setPathAttr :: proc(key, value: string, out: ^__PathAttr) -> SvgError {
	switch key {
	case "d":
		out.d = value
	case "fill":
		out.fill = value
	case "fill-opacity":
		_parseNumberAttr(value, &out.fillOpacity) or_return
	case "stroke":
		out.stroke = value
	case "stroke-opacity":
		_parseNumberAttr(value, &out.strokeOpacity) or_return
	case "stroke-width":
		_parseNumberAttr(value, &out.strokeWidth) or_return
	case "transform":
		out.transform = value
	}
	return nil
}

@(private)
_parseStyleAttr :: proc(style: string, out: ^__PathAttr) -> SvgError {
	i := 0
	for i < len(style) {
		declStart := i
		for i < len(style) && style[i] != ';' {
			i += 1
		}
		decl := strings.trim_space(style[declStart:i])
		if i < len(style) && style[i] == ';' do i += 1
		if len(decl) == 0 do continue

		colon := strings.index_byte(decl, ':')
		if colon < 0 do continue

		key := strings.trim_space(decl[:colon])
		value := strings.trim_space(decl[colon + 1:])
		_setPathAttr(key, value, out) or_return
	}
	return nil
}

@(private)
_parseSvgColor :: proc(
	colorText: string,
	opacity: Maybe(f32),
) -> (
	color: linalg.Vector4f32,
	ok: bool,
	err: SvgError = nil,
) {
	value := strings.trim_space(colorText)
	if len(value) == 0 || _isNoneValue(value) {
		return
	}

	clr: pv.plutovg_color_t
	consumed, parseErr := pv.plutovg_color_parse_string(&clr, value)
	if parseErr != nil {
		err = parseErr
		return
	}
	if consumed == 0 {
		err = .INVALID_NODE
		return
	}

	alpha := clr.a
	if o, has := opacity.?; has {
		alpha *= math.clamp(o, 0.0, 1.0)
	}
	color = linalg.Vector4f32{clr.r, clr.g, clr.b, math.clamp(alpha, 0.0, 1.0)}
	ok = color.w > 0.0
	return
}

@(private)
_resetContour :: proc(
	pts: ^[dynamic]linalg.Vector2f32,
	curves: ^[dynamic]bool,
	arena: mem.Allocator,
) {
	pts^ = make([dynamic]linalg.Vector2f32, arena)
	curves^ = make([dynamic]bool, arena)
}

@(private)
_normalizeContourWinding :: proc(
	ptsOut: ^[dynamic][]linalg.Vector2f32,
	curvesOut: ^[dynamic][]bool,
	arena: mem.Allocator,
) -> SvgError {
	if len(ptsOut^) != len(curvesOut^) do return .INVALID_NODE

	for i in 0 ..< len(ptsOut^) {
		pts := ptsOut^[i]
		curves := curvesOut^[i]
		if len(pts) < 3 do continue
		if len(pts) != len(curves) do return .INVALID_NODE

		isHole := geometry.IsHoleContour(i, ptsOut^[:])
		orientation := linalg_ex.GetPolygonOrientation(pts)
		needReverse :=
			(!isHole && orientation != .CounterClockwise) ||
			(isHole && orientation != .Clockwise)
		if needReverse {
			err: geometry.ShapeError
			ptsOut^[i], curvesOut^[i], err = geometry.ReverseShapeCloseCurve(pts, curves, arena)
			if err != nil do return .INVALID_NODE
		}
	}
	return nil
}

@(private)
_finalizeContour :: proc(
	inoutPts: ^[dynamic]linalg.Vector2f32,
	inoutCurves: ^[dynamic]bool,
	isClosed: bool,
	ptsOut: ^[dynamic][]linalg.Vector2f32,
	curvesOut: ^[dynamic][]bool,
	arena: mem.Allocator,
) -> SvgError {
	if len(inoutPts^) == 0 {
		_resetContour(inoutPts, inoutCurves, arena)
		return nil
	}
	if len(inoutPts^) != len(inoutCurves^) {
		return .INVALID_NODE
	}

	//if same first and last point of a closed contour, remove the last point to avoid duplicate in geometry.shapes_compute_polygon.
	if isClosed &&
	   len(inoutPts^) > 1 &&
	   !inoutCurves^[len(inoutCurves^) - 1] &&
	   inoutPts^[len(inoutPts^) - 1] == inoutPts^[0] {
		non_zero_resize_dynamic_array(inoutPts, len(inoutPts^) - 1) or_return
		non_zero_resize_dynamic_array(inoutCurves, len(inoutCurves^) - 1) or_return
	}

	// geometry.shapes_compute_polygon expects the first point flag to be non-curve.
	// If needed, rotate contour start to the first non-curve point while preserving pairing.
	if isClosed && len(inoutCurves^) > 0 && inoutCurves^[0] {
		start := -1
		for i in 0 ..< len(inoutCurves^) {
			if !inoutCurves^[i] {
				start = i
				break
			}
		}
		if start < 0 {
			return .INVALID_NODE
		}
		if start > 0 {
			rotPts: [dynamic]linalg.Vector2f32 = make([dynamic]linalg.Vector2f32, arena)
			rotCurves: [dynamic]bool = make([dynamic]bool, arena)
			non_zero_reserve(&rotPts, len(inoutPts^)) or_return
			non_zero_reserve(&rotCurves, len(inoutCurves^)) or_return
			for i in 0 ..< len(inoutPts^) {
				src := (start + i) % len(inoutPts^)
				non_zero_append(&rotPts, inoutPts^[src]) or_return
				non_zero_append(&rotCurves, inoutCurves^[src]) or_return
			}
			inoutPts^ = rotPts
			inoutCurves^ = rotCurves
		}
	}

	if isClosed && len(inoutPts^) >= 2 {
		non_zero_append(ptsOut, inoutPts^[:]) or_return
		non_zero_append(curvesOut, inoutCurves^[:]) or_return
	}

	_resetContour(inoutPts, inoutCurves, arena)
	return nil
}

@(private)
_pathToClosedContours :: proc(
	path: ^pv.plutovg_path_t,
	ptsOut: ^[dynamic][]linalg.Vector2f32,
	curvesOut: ^[dynamic][]bool,
	arena: mem.Allocator,
) -> SvgError {
	currentPts: [dynamic]linalg.Vector2f32
	currentCurves: [dynamic]bool
	_resetContour(&currentPts, &currentCurves, arena)

	it: pv.plutovg_path_iterator_t
	pv.plutovg_path_iterator_init(&it, path)
	for pv.plutovg_path_iterator_has_next(&it) {
		points: [3]pv.plutovg_point_t
		cmd := pv.plutovg_path_iterator_next(&it, &points)

		switch cmd {
		case .PLUTOVG_PATH_COMMAND_MOVE_TO:
			_finalizeContour(
				&currentPts,
				&currentCurves,
				true,
				ptsOut,
				curvesOut,
				arena,
			) or_return
			non_zero_append(&currentPts, _toGeomPoint(points[0])) or_return
			non_zero_append(&currentCurves, false) or_return
		case .PLUTOVG_PATH_COMMAND_LINE_TO:
			if len(currentPts) == 0 do return .INVALID_NODE
			non_zero_append(&currentPts, _toGeomPoint(points[0])) or_return
			non_zero_append(&currentCurves, false) or_return
		case .PLUTOVG_PATH_COMMAND_CUBIC_TO:
			if len(currentPts) == 0 do return .INVALID_NODE
			non_zero_append(
				&currentPts,
				_toGeomPoint(points[0]),
				_toGeomPoint(points[1]),
				_toGeomPoint(points[2]),
			) or_return
			non_zero_append(&currentCurves, true, true, false) or_return
		case .PLUTOVG_PATH_COMMAND_CLOSE:
			if len(currentPts) == 0 do continue
			_finalizeContour(
				&currentPts,
				&currentCurves,
				true,
				ptsOut,
				curvesOut,
				arena,
			) or_return
		case:
			return .UNSUPPORTED_FEATURE
		}
	}

	// Trailing subpath can be open in source data, but fill semantics still close it.
	_finalizeContour(&currentPts, &currentCurves, true, ptsOut, curvesOut, arena) or_return
	return nil
}

@(private)
_pathElementToShapeNode :: proc(
	e: ^xml.Element,
	arena: mem.Allocator,
) -> (
	node: geometry.ShapeNode,
	ok: bool,
	err: SvgError = nil,
) {
	attr := __PathAttr{}
	for a in e.attribs {
		switch a.key {
		case "style":
			_parseStyleAttr(a.val, &attr) or_return
		case:
			_setPathAttr(strings.trim_space(a.key), strings.trim_space(a.val), &attr) or_return
		}
	}

	if attr.d == nil do return

	fillColor := linalg.Vector4f32{0.0, 0.0, 0.0, 1.0}
	hasFill := true
	if fill, has := attr.fill.?; has {
		fillColor, hasFill, err = _parseSvgColor(fill, attr.fillOpacity)
		if err != nil do return
	} else if a, has := attr.fillOpacity.?; has {
		fillColor.w = math.clamp(a, 0.0, 1.0)
		hasFill = fillColor.w > 0.0
	}
	if !hasFill do return

	strokeColor := linalg.Vector4f32{}
	hasStroke := false
	if stroke, has := attr.stroke.?; has {
		strokeColor, hasStroke, err = _parseSvgColor(stroke, attr.strokeOpacity)
		if err != nil do return
	}
	strokeWidth: f32 = 0.0
	if hasStroke {
		strokeWidth = 1.0
		if w, has := attr.strokeWidth.?; has {
			strokeWidth = math.max(w, 0.0)
		}
	}

	path := pv.plutovg_path_create()
	if path == nil {
		err = .INVALID_NODE
		return
	}
	defer pv.plutovg_path_destroy(path)

	d := strings.trim_space(attr.d.?)
	if len(d) == 0 do return
	dOk, dErr := pv.plutovg_path_parse_string(path, d)
	if dErr != nil {
		err = dErr
		return
	}
	if !dOk {
		err = .INVALID_NODE
		return
	}

	if t, has := attr.transform.?; has {
		t = strings.trim_space(t)
		if len(t) > 0 {
			m: pv.plutovg_matrix_t
			tOk, tErr := pv.plutovg_matrix_parse_string(&m, t)
			if tErr != nil {
				err = tErr
				return
			}
			if !tOk {
				err = .INVALID_NODE
				return
			}
			pv.plutovg_path_transform(path, &m)
		}
	}

	pts: [dynamic][]linalg.Vector2f32 = make([dynamic][]linalg.Vector2f32, arena)
	curves: [dynamic][]bool = make([dynamic][]bool, arena)
	_pathToClosedContours(path, &pts, &curves, arena) or_return
	_normalizeContourWinding(&pts, &curves, arena) or_return
	if len(pts) == 0 do return

	node = geometry.ShapeNode {
		pts          = pts[:],
		isCurves    = curves[:],
		color        = fillColor,
		strokeColor = strokeColor,
		thickness    = strokeWidth,
		isClosed    = true,
	}
	ok = true
	return
}

@(private)
_parseSvgElement :: proc(
	xmlDoc: ^xml.Document,
	idx: xml.Element_ID,
	nodes: ^[dynamic]geometry.ShapeNode,
	arena: mem.Allocator,
) -> SvgError {
	e := &xmlDoc.elements[idx]
	if e.ident == "path" {
		node, ok, pathErr := _pathElementToShapeNode(e, arena)
		if pathErr != nil do return pathErr
		if ok {
			non_zero_append(nodes, node) or_return
		}
	}

	for v in e.value {
		switch value in v {
		case xml.Element_ID:
			_parseSvgElement(xmlDoc, value, nodes, arena) or_return
		case string:
		}
	}
	return nil
}

initParse :: proc(
	svgData: []u8,
	allocator: mem.Allocator = context.allocator,
) -> (
	parser: SvgParser,
	err: SvgError = nil,
) {
	if len(svgData) == 0 {
		err = .INVALID_NODE
		return
	}

	parser = {}
	mem.dynamic_arena_init(&parser.__arena, allocator, allocator)
	arena := mem.dynamic_arena_allocator(&parser.__arena)
	parser.arenaAllocator = arena
	defer if err != nil {
		deinit(&parser)
	}

	xmlDoc, xmlErr := xml.parse(svgData, allocator = context.temp_allocator)
	if xmlErr != nil {
		err = xmlErr
		return
	}
	defer xml.destroy(xmlDoc, context.temp_allocator)

	nodes: [dynamic]geometry.ShapeNode = make([dynamic]geometry.ShapeNode, arena)
	_parseSvgElement(xmlDoc, 0, &nodes, arena) or_return

	parser.shapes = geometry.Shapes {
		nodes = nodes[:],
	}
	return
}
