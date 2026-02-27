#ifndef GEOMETRY_H
#define GEOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct { float x, y; } Geometry_Vec2;
typedef struct { float x, y, z; } Geometry_Vec3;
typedef struct { float x, y, z, w; } Geometry_Vec4;

typedef struct {
    Geometry_Vec2 pos;
    Geometry_Vec3 uvw;
    Geometry_Vec4 color;
} Geometry_Vertex;

typedef struct {
    Geometry_Vertex* vertices;
    uint32_t* indices;
    size_t vertex_count;
    size_t index_count;
} Geometry_RawShape;

typedef struct {
    Geometry_Vec2* pts;
    size_t pts_count;
    uint32_t* curve_pts_ids;
    size_t curve_pts_ids_count;
    Geometry_Vec4 color;
    Geometry_Vec4 stroke_color;
    double thickness;
    bool is_closed;
} Geometry_Node;

typedef struct {
    Geometry_Node* nodes;
    size_t node_count;
} Geometry_Shapes;

/* Fixed-point: 24 fractional bits, 1.0 = 1<<24 = 16777216 */
typedef int64_t Geometry_Fixed;
#define GEOMETRY_FIXED_ONE (1LL << 24)

typedef struct {
    Geometry_Fixed x, y;
} Geometry_Vec2_Fixed;

typedef struct {
    Geometry_Fixed pos[2];
    Geometry_Vec3 uvw;
    Geometry_Vec4 color;
} Geometry_Vertex_Fixed;

typedef struct {
    Geometry_Vertex_Fixed* vertices;
    uint32_t* indices;
    size_t vertex_count;
    size_t index_count;
} Geometry_RawShape_Fixed;

typedef struct {
    Geometry_Vec2_Fixed* pts;
    size_t pts_count;
    uint32_t* curve_pts_ids;
    size_t curve_pts_ids_count;
    Geometry_Vec4 color;
    Geometry_Vec4 stroke_color;
    Geometry_Fixed thickness;
    bool is_closed;
} Geometry_Node_Fixed;

typedef struct {
    Geometry_Node_Fixed* nodes;
    size_t node_count;
} Geometry_Shapes_Fixed;

typedef enum {
    Geometry_Error_None = 0,
    Geometry_Error_IsPointNotLine = 1,
    Geometry_Error_EmptyPolygon = 2,
    Geometry_Error_EmptyColor = 3,
    Geometry_Error_Allocator = 4,
} Geometry_Error;

typedef struct {
    void* procedure;
    void* data;
} Odin_Allocator;

void geometry_set_alloc_func(void* (*alloc)(size_t));
void geometry_set_calloc_func(void* (*calloc)(size_t, size_t));
void geometry_set_realloc_func(void* (*realloc)(void*, size_t));
void geometry_set_free_func(void (*free)(void*));

Geometry_Error geometry_shapes_compute_polygon(const Geometry_Shapes* input, Geometry_RawShape* out);
void geometry_raw_shape_free(Geometry_RawShape* shape);

/* Fixed-point helpers */
Geometry_Fixed geometry_fixed_from_f64(double v);
double geometry_fixed_to_f64(Geometry_Fixed v);

Geometry_Error geometry_shapes_compute_polygon_fixed(const Geometry_Shapes_Fixed* input, Geometry_RawShape_Fixed* out);
void geometry_raw_shape_free_fixed(Geometry_RawShape_Fixed* shape);

#ifdef __cplusplus
}
#endif

#endif /* GEOMETRY_H */
