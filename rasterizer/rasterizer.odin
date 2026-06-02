package rasterizer

import ".."
import "base:runtime"
import "core:c"
import "core:math/linalg"
import "core:mem"
import pv "shared:clibs/plutovg"
import "shared:utils_private"

__RasterizerError :: enum {
	INVALID_SIZE,
	SURFACE_CREATE_FAILED,
	CANVAS_CREATE_FAILED,
}

RasterizerError :: union #shared_nil {
	__RasterizerError,
	runtime.Allocator_Error,
}

PixelFormat :: enum u8 {
	// Matches plutovg surface memory layout.
	ARGB8_PREMULTIPLIED,
}

RasterizedPixels :: struct {
	width:     int,
	height:    int,
	stride:    int, // bytes per row
	format:    PixelFormat,
	pixels:    []u8,
	allocator: runtime.Allocator,
}

rasterizedPixelsFree :: proc(self: ^RasterizedPixels) {
	if self == nil do return
	if self.pixels != nil do delete(self.pixels, self.allocator)
	self^ = {}
}

@(private = "file")
_appendContourPath :: proc(
	canvas: ^pv.plutovg_canvas_t,
	pts: []linalg.Vector2f32,
	curveFlags: []bool,
	isClosed: bool,
) {
	if canvas == nil || len(pts) == 0 do return

	pv.plutovg_canvas_move_to(canvas, pts[0].x, pts[0].y)

	last := len(pts) - 1
	if last <= 0 {
		if isClosed do pv.plutovg_canvas_close_path(canvas)
		return
	}

	i := 0
	for {
		if !isClosed && i >= last do break

		next := i + 1
		if isClosed do next = (i + 1) % len(pts)
		if next >= len(pts) do break

		if next < len(curveFlags) && curveFlags[next] {
			next2 := next + 1
			if isClosed do next2 = (next + 1) % len(pts)
			if next2 >= len(pts) do break

			if next2 < len(curveFlags) && curveFlags[next2] {
				next3 := next2 + 1
				if isClosed do next3 = (next2 + 1) % len(pts)
				if next3 >= len(pts) do break

				pv.plutovg_canvas_cubic_to(
					canvas,
					pts[next].x,
					pts[next].y,
					pts[next2].x,
					pts[next2].y,
					pts[next3].x,
					pts[next3].y,
				)
				i = next3
			} else {
				pv.plutovg_canvas_quad_to(
					canvas,
					pts[next].x,
					pts[next].y,
					pts[next2].x,
					pts[next2].y,
				)
				i = next2
			}
		} else {
			pv.plutovg_canvas_line_to(canvas, pts[next].x, pts[next].y)
			i = next
		}

		if isClosed && i == 0 do break
	}

	if isClosed do pv.plutovg_canvas_close_path(canvas)
}

shapesToPixels :: proc(
	src: geometry.Shapes,
	width, height: int,
	allocator := context.allocator,
) -> (
	out: RasterizedPixels,
	err: RasterizerError = nil,
) {
	if width <= 0 || height <= 0 {
		err = .INVALID_SIZE
		return
	}

	surface := pv.plutovg_surface_create(c.int(width), c.int(height))
	if surface == nil {
		err = .SURFACE_CREATE_FAILED
		return
	}
	defer pv.plutovg_surface_destroy(surface)

	canvas := pv.plutovg_canvas_create(surface)
	if canvas == nil {
		err = .CANVAS_CREATE_FAILED
		return
	}
	defer pv.plutovg_canvas_destroy(canvas)

	clear := pv.plutovg_color_t {
		r = 0.0,
		g = 0.0,
		b = 0.0,
		a = 0.0,
	}
	pv.plutovg_surface_clear(surface, &clear)
	pv.plutovg_canvas_set_fill_rule(canvas, .PLUTOVG_FILL_RULE_NON_ZERO)

	for node in src.nodes {
		if len(node.pts) == 0 do continue

		pv.plutovg_canvas_new_path(canvas)
		for contour, cidx in node.pts {
			curveFlags: []bool = nil
			if node.isCurves != nil && cidx < len(node.isCurves) {
				curveFlags = node.isCurves[cidx]
			}
			_appendContourPath(canvas, contour, curveFlags, node.isClosed)
		}

		hasFill := node.color.w > 0.0
		hasStroke := node.strokeColor.w > 0.0 && node.thickness > 0.0

		if hasFill {
			pv.plutovg_canvas_set_rgba(
				canvas,
				node.color.x,
				node.color.y,
				node.color.z,
				node.color.w,
			)
			if hasStroke {
				pv.plutovg_canvas_fill_preserve(canvas)
			} else {
				pv.plutovg_canvas_fill(canvas)
			}
		}

		if hasStroke {
			pv.plutovg_canvas_set_rgba(
				canvas,
				node.strokeColor.x,
				node.strokeColor.y,
				node.strokeColor.z,
				node.strokeColor.w,
			)
			pv.plutovg_canvas_set_line_width(canvas, node.thickness)
			pv.plutovg_canvas_stroke(canvas)
		}
	}

	stride := int(pv.plutovg_surface_get_stride(surface))
	pixelBytes := stride * height
	if pixelBytes < 0 {
		err = .INVALID_SIZE
		return
	}

	pixels := utils_private.makeNonZeroedSlice([]u8, pixelBytes, allocator) or_return
	mem.copy_non_overlapping(raw_data(pixels), pv.plutovg_surface_get_data(surface), pixelBytes)

	out = RasterizedPixels {
		width     = width,
		height    = height,
		stride    = stride,
		format    = .ARGB8_PREMULTIPLIED,
		pixels    = pixels,
		allocator = allocator,
	}
	return
}
