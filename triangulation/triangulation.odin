package triangulation

import "base:intrinsics"
import "base:runtime"
import "core:mem"
import tess "shared:clibs/libtess2"
import utils "shared:utils_private"
import "shared:utils_private/library"

//=============================================================================
// Error
//=============================================================================
TrianguateError :: union #shared_nil {
	__TrianguateError,
	runtime.Allocator_Error,
}

__TrianguateError :: enum {
	FAILED,
	TOO_FEW_POINTS,
}

//=============================================================================
// Public API
//=============================================================================
TrianguatePolygons :: proc(
	poly: [][][2]f32,
	allocator := context.allocator,
	offset: u32 = 0,
) -> (
	outIndices: []u32,
	err: TrianguateError,
) {
	context.allocator = allocator

	outIndices = tess.triangulate(poly, offset, context.allocator)
	if outIndices == nil do return nil, .FAILED
	if len(outIndices) < 3 do return nil, .FAILED
	return
}
