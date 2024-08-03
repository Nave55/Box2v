module box2v

import math

#flag -I @VMODROOT\include\box2d

#include "box2d.h"

#flag -L @VMODROOT\include\bin

$if windows {
	#flag -l:libbox2d.a
}

// start math_functions

const pi = 3.14159265359

// 2D vector
// This can be used to represent a point or free vector
@[typedef]
struct C.b2Vec2 {
pub mut:
	// coordinates
	x f32
	y f32
}

pub type Vec2 = C.b2Vec2

// 2D rotation
// This is similar to using a complex number for rotation
@[typedef]
struct C.b2Rot {
pub mut:
	// cosine and sine
	c f32
	s f32
} 

pub type Rot = C.b2Rot

// A 2D rigid transform
@[typedef]
struct C.b2Transform {
pub mut:
	p Vec2
	q Rot
}

pub type Transform = C.b2Transform

// A 2-by-2 Matrix
@[typedef]
struct C.b2Mat22 {
pub mut:
	// columns
	cx Vec2
	cy Vec2
}

pub type Mat22 = C.b2Mat22

// Axis-aligned bounding box
@[typedef]
struct C.b2AABB {
pub mut:
	lowerBound Vec2 
	upperBound Vec2
}

pub type AABB = C.b2AABB

pub const vec2_zero = Vec2{}
pub const rot_identity = Rot{1.0, 0.0}
pub const transform_identity = Transform{Vec2{0.0, 0.0}, Rot{1.0, 0.0}}
pub const mat22_zero = Mat22{}

// return the minimum of two floats
fn C.b2MinFloat(a f32, b f32) f32
@[inline]
pub fn min_float(a f32, b f32) f32 {
	return C.b2MinFloat(a, b)
}

// @return the maximum of two floats
fn C.b2MaxFloat(a f32, b f32) f32
@[inline]
pub fn max_float(a f32, b f32) f32 {
	return C.b2MaxFloat(a, b)
}

// return the absolute value of a f32
fn C.b2AbsFloat(a f32) f32
@[inline]
pub fn abs_float(a f32) f32 {
	return C.b2AbsFloat(a)
}

// return a clamped between a lower and upper bound
fn C.b2ClampFloat(a f32, lower f32, upper f32) f32
@[inline]
pub fn clamp_float(a f32, lower f32, upper f32) f32 {
	return C.b2ClampFloat(a, lower, upper)
}

// return the minimum of two integers
fn C.b2MinInt(a int, b int) int
@[inline]
pub fn min_int(a int, b int) int {
	return C.b2MinInt(a, b)
}

// return the maximum of two integers
fn C.b2MaxInt(a int, b int) int
@[inline]
pub fn max_int(a int, b int) int {
	return C.b2MaxInt(a, b)
}

// return the absolute value of an integer
fn C.b2AbsInt(a int) int
@[inline]
pub fn abs_int(a int) int {
	return C.b2AbsInt(a)
}

// return an integer clamped between a lower and upper bound
fn C.b2ClampInt(a int, lower int, upper int) int
@[inline]
pub fn clamp_int(a int, lower int, upper int) int {
	return C.b2ClampInt(a, lower, upper)
}

// Vector dot product
fn C.b2Dot(a Vec2, b Vec2) f32
@[inline]
pub fn dot(a Vec2, b Vec2) f32 {
	return C.b2Dot(a, b)
}

// Vector cross product. In 2D this yields a scalar.
fn C.b2Cross(a Vec2, b Vec2) f32
@[inline]
pub fn cross(a Vec2, b Vec2) f32 {
	return C.b2Cross(a, b)
}

// Perform the cross product on a vector and a scalar. In 2D this produces a vector.
@[inline]
pub fn cross_vs(v Vec2, s f32) &Vec2 {
	return &Vec2{s * v.y, -s * v.x}
}

// Perform the cross product on a scalar and a vector. In 2D this produces a vector.
@[inline]
pub fn cross_sv(s f32, v Vec2) &Vec2 {
	return &Vec2{-s * v.y, s * v.x}
}

// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
@[inline]
pub fn left_perp(v Vec2) &Vec2 {
	return &Vec2{-v.y, v.x}
}

// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
@[inline]
pub fn right_perp(v Vec2) &Vec2 {
	return &Vec2{v.y, -v.x}
}

// Vector addition
@[inline]
pub fn add(a Vec2, b Vec2) &Vec2 {
	return &Vec2{a.x + b.x, a.y + b.y}
}

// Vector subtraction
@[inline]
pub fn sub(a Vec2, b Vec2) &Vec2 {
	return &Vec2{a.x - b.x, a.y - b.y}
}

// Vector negation
@[inline]
pub fn neg(a Vec2) &Vec2 {
	return &Vec2{-a.x, -a.y}
}

// Vector linear interpolation
// https://fgiesen.wordpress.com/2012/08/
@[inline]
pub fn lerp(a Vec2, b Vec2, t f32) &Vec2 {
	return &Vec2{(1.0 - t) * a.x + t * b.x, (1.0 - t) * a.y + t * b.y}
}

// Component-wise multiplication
@[inline]
pub fn mul(a Vec2, b Vec2) &Vec2 {
	return &Vec2{a.x * b.x, a.y * b.y}
}

// Multiply a scalar and vector
@[inline]
pub fn mul_sv(s f32, v Vec2) &Vec2 {
	return &Vec2{s * v.x, s * v.y}
}

// a + s * b
@[inline]
pub fn mul_add(a Vec2, s f32, b Vec2) &Vec2 {
	return &Vec2{a.x + s * b.x, a.y + s * b.y}
}

// a - s * b
@[inline]
pub fn mul_sub(a Vec2, s f32, b Vec2) &Vec2 {
	return &Vec2{a.x - s * b.x, a.y - s * b.y}
}

// Component-wise absolute vector
@[inline]
pub fn abs(a Vec2) &Vec2 {
	mut b := Vec2{}
	b.x = abs_float(a.x)
	b.y = abs_float(a.y)
	return &b
}

// Component-wise minimum vector
@[inline]
pub fn min(a Vec2, b Vec2) &Vec2 {
	mut c := Vec2{}
	c.x = min_float(a.x, b.x)
	c.y = min_float(a.y, b.y)
	return &c
}

// Component-wise maximum vector
@[inline]
pub fn max(a Vec2, b Vec2) &Vec2 {
	mut c := Vec2{}
	c.x = max_float(a.x, b.x)
	c.y = max_float(a.y, b.y)
	return &c
}

// Component-wise clamp vector v into the range [a, b]
@[inline]
pub fn clamp(v Vec2, a Vec2, b Vec2) &Vec2 {
	mut c := Vec2{}
	c.x = clamp_float(v.x, a.x, b.x)
	c.y = clamp_float(v.y, a.y, b.y)
	return &c
}

// Get the length of this vector (the norm)
fn C.b2Length(v Vec2) f32
@[inline]
pub fn length(v Vec2) f32 {
	return C.b2Length(v)
}

// Get the length squared of this vector
fn C.b2LengthSquared(v Vec2) f32
@[inline]
pub fn length_squared(v Vec2) f32 {
	return C.b2LengthSquared(v)
}

// Get the distance between two points
fn C.b2Distance(a Vec2, b Vec2) f32
@[inline]
pub fn distance(a Vec2, b Vec2) f32 {
	return C.b2Distance(a, b)
}

// Get the distance squared between two points
fn C.b2DistanceSquared(a Vec2, b Vec2) f32
@[inline]
pub fn distance_squared(a Vec2, b Vec2) f32 {
	return C.b2DistanceSquared(a, b)
}

// Set using an angle in radians
@[inline]
pub fn make_rot(angle f32) &Rot {
	// todo determinism
	q := Rot{math.cosf(angle), math.sinf(angle)}
	return &q
}

// Normalize rotation
@[inline]
pub fn normalize_rot(q Rot) &Rot {
	mut mag := math.sqrtf(q.s * q.s + q.c * q.c)
	mut inv_mag := if mag > 0.0 {f32(1.0)} else {f32(0.0)}
	qn := Rot{q.c * inv_mag, q.s * inv_mag}
	return &qn

}

// Is this rotation normalized?
fn C.b2IsNormalized(q Rot) bool
@[inline]
pub fn is_normalized(q Rot) bool {
	return C.b2IsNormalized(q)
}

// Normalized linear interpolation
// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
@[inline]
pub fn n_lerp(q1 Rot, q2 Rot, t f32) &Rot {
	omt := f32(1.0 - t)
	q := Rot{omt * q1.c + t * q2.c, 
	                  omt * q1.s + t * q2.s}
	return normalize_rot(q)
}

// Integration rotation from angular velocity
//	q1 initial rotation
//	deltaAngle the angular displacement in radians
@[inline]
pub fn integration_rotation(q1 Rot, delta_angle f32) &Rot {
	q2 := Rot{q1.c - delta_angle * q1.s, q1.s + delta_angle * q1.c}
	mag := f32(math.sqrtf(q2.s * q2.s + q2.c * q2.c))
	inv_mag := if mag > 0.0 {1.0 / mag} else {0.0}
	qn := Rot{q2.c * inv_mag, q2.s * inv_mag}
	return &qn
}

// Compute the angular velocity necessary to rotate between two rotations over a give time
//	q1 initial rotation
//	q2 final rotation
//	inv_h inverse time step
fn C.b2ComputeAngularVelocity(q1 Rot, q2 Rot, inv_h f32) f32
@[inline]
pub fn compute_angular_velocity(q1 Rot, q2 Rot, inv_h f32) f32 {
	return C.b2ComputeAngularVelocity(q1, q2, inv_h)
}

// Get the angle in radians
fn C.b2Rot_GetAngle(q Rot) f32
@[inline]
pub fn rot_get_angle(q Rot) f32 {
	return C.b2Rot_GetAngle(q)
}

// Get the x-axis
@[inline]
pub fn rot_get_x_axis(q Rot) &Vec2 {
	v := Vec2 {q.c, q.s}
	return &v
}

// Get the y-axis
@[inline]
pub fn rot_get_y_axis(q Rot) &Vec2 {
	v := Vec2{-q.s, q.c}
	return &v
}

// Multiply two rotations: q * r
@[inline]
pub fn mul_rot(q Rot, r Rot) &Rot {
	mut qr := Rot{} 
	qr.s = q.s * r.c + q.c * r.s
	qr.c = q.c * r.c - q.s * r.s
	return &qr
}

// Transpose multiply two rotations: qT * r
@[inline]
pub fn inv_mul_rot(q Rot, r Rot) &Rot {
	mut qr := Rot{} 
	qr.s = q.c * r.s - q.s * r.c
	qr.c = q.c * r.c + q.s * r.s
	return &qr
}

// relative angle between b and a (rot_b * inv(rot_a))// Transpose multiply two rotations: qT * r
fn C.b2RelativeAngle(b Rot, a Rot) f32
@[inline]
pub fn relative_angle(b Rot, a Rot) f32 {
	return C.b2RelativeAngle(b, a)
}

// Convert an angle in the range [-2*pi, 2*pi] into the range [-pi, pi]
fn C.b2UnwindAngle(angle f32) f32
@[inline]
pub fn unwind_angle(angle f32) f32 {
	return C.b2UnwindAngle(angle)
}

// Rotate a vector
@[inline]
pub fn rotate_vector(q Rot, v Vec2) &Vec2 {
	return &Vec2{q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y}
}

// Inverse Rotate a vector
@[inline]
pub fn inv_rotate_vector(q Rot, v Vec2) &Vec2 {
	return &Vec2{q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y}
}


// Transform a point (e.g. local space to world space)
@[inline]
pub fn transform_point(t Transform, p Vec2) &Vec2 {
	x := f32(t.q.c * p.x - t.q.s * p.y) + t.p.x
	y := f32(t.q.s * p.x + t.q.c * p.y) + t.p.y

	return &Vec2{x, y}
}


// Inbere Transform a point (e.g. local space to world space)
@[inline]
pub fn inv_transform_point(t Transform, p Vec2) &Vec2 {
	vx := f32(p.x - t.p.x)
	vy := f32(p.y - t.p.y)
	return &Vec2{t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy}

}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
@[inline]
pub fn mul_transforms(a Transform, b Transform) &Transform {
	mut c := Transform{}
	c.q = mul_rot(a.q, b.q)
	c.p = add(rotate_vector(a.q, b.p), a.p)
	return &c
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
@[inline]
pub fn inv_mul_transforms(a Transform, b Transform) &Transform {
	mut c := Transform{}  
	c.q = inv_mul_rot(a.q, b.q)
	c.p = inv_rotate_vector(a.q, sub(b.p, a.p))
	return &c
}

// Multiply a 2-by-2 matrix times a 2D vector
@[inline]
pub fn mul_mv(a Mat22, v Vec2) &Vec2 {
	u := Vec2{a.cx.x * v.x + a.cy.x * v.y,
	                    a.cx.y * v.x + a.cy.y * v.y}
	return &u
}

// Get the inverse of a 2-by-2 matrix
@[inline]
pub fn get_inverse_22(z Mat22) &Mat22 {
	a := z.cx.x
	b := z.cy.x
	c := z.cx.y
	d := z.cy.y
	mut det :=  a * d - b * c
	if det != 0.0 {
		det = 1.0 / det
	}

	y := Mat22{Vec2{det * d, -det * c},
		                  Vec2{-det * b, det * a}}
	return &y
}

// Solve A * x = b, where b is a column vector. This is more efficient
// than computing the inverse in one-shot cases.
@[inline]
pub fn solve_22(a Mat22, b Vec2) &Vec2 {
	a11 := f32(a.cx.x) 
	a12 := f32(a.cy.x) 
	a21 := f32(a.cx.y) 
	a22 := f32(a.cy.y)
	mut det := a11 * a22 - a12 * a21
	if det != 0.0 {
		det = 1.0 / det
	}
	x := Vec2{det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x)}
	return &x
}

// Does a fully contain b
fn C.b2AABB_Contains(a AABB, b AABB) bool
@[inline]
pub fn aabb_contains(a AABB, b AABB) bool {
	return C.b2AABB_Contains(a, b)
}

// Get the center of the AABB.
@[inline]
pub fn aabb_center(a AABB) &Vec2 {
	return &Vec2{0.5 * (a.lowerBound.x + a.upperBound.x), 0.5 * (a.lowerBound.y + a.upperBound.y)}
}

@[inline]
pub fn aabb_extents(a AABB) &Vec2 {
	return &Vec2{0.5 * (a.upperBound.x - a.lowerBound.x), 0.5 * (a.upperBound.y - a.lowerBound.y)}

}

@[inline]
pub fn aabb_union(a AABB, b AABB) &AABB {
	mut c := AABB{} 
	c.lowerBound.x = min_float(a.lowerBound.x, b.lowerBound.x)
	c.lowerBound.y = min_float(a.lowerBound.y, b.lowerBound.y)
	c.upperBound.x = max_float(a.upperBound.x, b.upperBound.x)
	c.upperBound.y = max_float(a.upperBound.y, b.upperBound.y)
	return &c
}

// Is this a valid number? Not NaN or infinity.
fn C.b2IsValid(a f32) bool
@[inline]
pub fn is_valid(a f32) bool {
	return C.b2IsValid(a)
}

// Is this a valid vector? Not NaN or infinity.
fn C.b2Vec2_IsValid(v Vec2) bool
@[inline]
pub fn vec2_is_valid(v Vec2) bool {
	return C.b2Vec2_IsValid(v)
}

// Is this a valid rotation? Not NaN or infinity. Is normalized.
fn C.b2Rot_IsValid(q Rot) bool
@[inline]
pub fn rot_is_valid(q Rot) bool {
	return C.b2Rot_IsValid(q)
}

// Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.
fn C.b2AABB_IsValid(aabb AABB) bool
@[inline]
pub fn aabb_is_valid(aabb AABB) bool {
	return C.b2AABB_IsValid(aabb)
}

// Is this a valid vector? Not NaN or infinity.
fn C.b2Vec2_IsValid(v Vec2) bool
@[inline]
pub fn is_vec2_valid(v Vec2) bool {
	return C.b2Vec2_IsValid(v)
}

// Convert a vector into a unit vector if possible, otherwise returns the zero vector.
fn C.b2Normalize(v Vec2) Vec2
@[inline]
pub fn normalize(v Vec2) Vec2 {
	return C.b2Normalize(v)
}

// Convert a vector into a unit vector if possible, otherwise asserts.
fn C.b2NormalizeChecked(v Vec2) Vec2
@[inline]
pub fn normalize_checked(v Vec2) Vec2 {
	return C.b2NormalizeChecked(v)
}

// Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
//	outputs the length.
fn C.b2GetLengthAndNormalize(length &f32, v Vec2) Vec2
@[inline]
pub fn get_length_and_normalize(length &f32, v Vec2) Vec2 {
	return C.b2GetLengthAndNormalize(length, v)
}

// Box2D bases all length units on meters, but you may need different units for your game.
// You can set this value to use different units. This should be done at application startup
//	and only modified once. Default value is 1.
//	@warning This must be modified before any calls to Box2D
fn C.b2SetLengthUnitsPerMeter(lengthUnits f32)
@[inline]
pub fn set_length_units_per_meter(lengthUnits f32) {
	C.b2SetLengthUnitsPerMeter(lengthUnits)
}

// overloads below

//overloads '+' operator to add 2 vecs
fn (a Vec2) + (b Vec2) &Vec2 {
	return &Vec2{a.x + b.x, a.y + b.y}
}

//overloads '-' operator to subract 2 vecs
fn (a Vec2) - (b Vec2) &Vec2 {
	return &Vec2{a.x - b.x, a.y - b.y}
}

//overloads '*' operator to multiply 2 vecs
fn (a Vec2) * (b Vec2) &Vec2 {
	return &Vec2{a.x * b.x, a.y * b.y}
}

//overloads '/' operator to divide 2 vecs
fn (a Vec2) / (b Vec2) &Vec2 {
	return &Vec2{a.x / b.x, a.y / b.y}
}

//overloads '==' operator to compare two vecs
fn (a Vec2) == (b Vec2) bool {
	return a.x == b.x && a.y == b.y
}

// end math_functions

// start base

// Prototype for user allocation function
pub type AllocFcn = fn(size u32, alignment int) voidptr
// Prototype for user free function
pub type FreeFcn = fn(mem voidptr)
// Prototype for the user assert callback. Return 0 to skip the debugger break.
pub type AssertFcn = fn(condition &char, fileName &char, lineNumber int) int

// This allows the user to override the allocation functions. These should be
//	set during application startup.
fn C.b2SetAllocator(allocFcn &AllocFcn, freeFcn &FreeFcn)
@[inline]
pub fn set_allocator(allocFcn &AllocFcn, freeFcn &FreeFcn) {
	C.b2SetAllocator(allocFcn, freeFcn)
}

// @return the total bytes allocated by Box2D
fn C.b2GetByteCount() int
@[inline]
pub fn get_byte_count() int {
	return C.b2GetByteCount()
}

// Override the default assert callback
//	assertFcn a non-null assert callback
fn C.b2SetAssertFcn(assertFcn &AssertFcn)
@[inline]
pub fn set_assert_fcn(assertFcn &AssertFcn) {
	C.b2SetAssertFcn(assertFcn)
}

// Get the current version of Box2D
fn C.b2GetVersion()
@[inline]
pub fn get_version() {
	C.b2GetVersion()
}

fn C.b2CreateTimer() Timer
@[inline]
pub fn create_timer() Timer {
	return C.b2CreateTimer()
}

fn C.b2GetTicks(timer &Timer) i64
@[inline]
pub fn get_ticks(timer &Timer) i64 {
	return C.b2GetTicks(timer)
}

fn C.b2GetMilliseconds(timer &Timer) f32
@[inline]
pub fn get_milli_seconds(timer &Timer) f32 {
	return C.b2GetMilliseconds(timer)
}

fn C.b2GetMillisecondsAndReset(timer &Timer) f32
@[inline]
pub fn get_milli_seconds_and_reset(timer &Timer) f32 {
	return C.b2GetMillisecondsAndReset(timer)
}

fn C.b2SleepMilliseconds(milliseconds int)
@[inline]
pub fn sleep_milliseconds(milliseconds int) {
	C.b2SleepMilliseconds(milliseconds)
}

fn C.b2Yield() 
@[inline]
pub fn yield() {
	C.b2Yield()
}

// Version numbering scheme.
// See https://semver.org/
@[typedef]
struct C.b2Version {
pub mut:
	// coordinates
	major int
	minor int
	revision int
}

pub type Version = C.b2Version

// to do. figure out how to implement linux and mac versions

// for windows
// Timer for profiling. This has platform specific code and may not work on every platform.
@[typedef]
struct C.b2Timer {
pub mut:
	// coordinate
	start i64
}

pub type Timer = C.b2Timer

// end base

// start id 

// World id references a world instance. This should be treated as an opaque handle.
@[typedef]
struct C.b2WorldId {
pub mut:
	index1 u16
	revision u16
}

pub type WorldId = C.b2WorldId

// Body id references a body instance. This should be treated as an opaque handle.
@[typedef]
struct C.b2BodyId {
pub mut:
	index1 int
	world0 u16
	revision u16
}

pub type BodyId = C.b2BodyId

// Shape id references a shape instance. This should be treated as an opaque handle.
@[typedef]
struct C.b2ShapeId {
pub mut:
	index1 int
	world0 u16
	revision u16
}

pub type ShapeId = C.b2ShapeId

// Joint id references a joint instance. This should be treated as an opaque handle.
@[typedef]
struct C.b2JointId {
pub mut:
	index1 int
	world0 u16
	revision u16
}

pub type JointId = C.b2JointId

// Chain id references a chain instances. This should be treated as an opaque handle.
@[typedef]
struct C.b2ChainId {
pub mut:
	index1 int
	world0 u16
	revision u16
}

pub type ChainId = C.b2ChainId

pub const null_world_id =  WorldId{0, 0}
pub const null_body_id = BodyId{0, 0, 0}
pub const null_shape_id = ShapeId{0, 0, 0}
pub const null_joint_id = JointId{0, 0, 0}
pub const null_chain_id = ChainId{0, 0, 0}

@[inline]
fn is_null[T](id T) bool {
	return id.index1 == 0
}

@[inline]
fn is_non_null[T](id T) bool {
	return id.index1 != 0
}

@[inline]
fn id_equals[T](id1 T, id2 T) bool {
	return id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.revision == id2.revision
}

// end id

// start collision

const max_polygon_vertices = 8

// Low level ray-cast input data
@[typedef]
struct C.b2RayCastInput {
pub mut:
	// Start point of the ray cast
	origin Vec2

	// Translation of the ray cast
	translation Vec2

	// The maximum fraction of the translation to consider, typically 1
	maxFraction f32
}

pub type RayCastInput = C.b2RayCastInput

// Low level shape cast input in generic form. This allows casting an arbitrary point
//	cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
//	A capsule is two points with a non-zero radius. A box is four points with a zero radius.
@[typedef]
struct C.b2ShapeCastInput {
pub mut:
	// A point cloud to cast
	points [max_polygon_vertices]Vec2

	// The number of points
	count int

	// The radius around the point cloud
	radius f32

	// The translation of the shape cast
	translation Vec2 

	// The maximum fraction of the translation to consider, typically 1
	maxFraction f32
}

pub type ShapeCastInput = C.b2ShapeCastInput


// Low level ray-cast or shape-cast output data
@[typedef]
struct C.b2CastOutput {
pub mut:
	// The surface normal at the hit point
	normal Vec2

	// The surface hit point
	point Vec2

	// The fraction of the input translation at collision
	fraction f32

	// The number of iterations used
	iterations int

	// Did the cast hit?
	hit bool
}

pub type CastOutput = C.b2CastOutput

// This holds the mass data computed for a shape.
@[typedef]
struct C.b2MassData {
pub mut:
	// The mass of the shape, usually in kilograms.
	mass f32

	// The position of the shape's centroid relative to the shape's origin.
	center Vec2 

	// The rotational inertia of the shape about the local origin.
	rotationalInertia f32
}

pub type MassData = C.b2MassData

// A solid circle
@[typedef]
struct C.b2Circle {
pub mut:
	// The local center
	center Vec2

	// The radius
	radius f32
}

pub type Circle = C.b2Circle

// A solid capsule can be viewed as two semicircles connected
//	by a rectangle.
@[typedef]
struct C.b2Capsule {
	// Local center of the first semicircle
	center1 Vec2
	
	// Local center of the second semicircle
	center2 Vec2

	// The radius of the semicircles
	radius f32
}

pub type Capsule = C.b2Capsule

// A solid convex polygon. It is assumed that the interior of the polygon is to
// the left of each edge.
// Polygons have a maximum number of vertices equal to maxPolygonVertices.
// In most cases you should not need many vertices for a convex polygon.
//	@warning DO NOT fill this out manually, instead use a helper function like
//	b2MakePolygon or b2MakeBox.
@[typedef] 
struct C.b2Polygon {
	// The polygon vertices
	vertices [max_polygon_vertices]Vec2

	// The outward normal vectors of the polygon sides
	normals [max_polygon_vertices]Vec2

	// The centroid of the polygon
	centroid Vec2

	// The external radius for rounded polygons
	radius f32

	// The number of polygon vertices
	count int
}

pub type Polygon = C.b2Polygon

// A line segment with two-sided collision.
@[typedef] 
struct C.b2Segment {
	// The first point
	point1 Vec2

	// The second point
	point2 Vec2
} 

pub type Segment = C.b2Segment

// A smooth line segment with one-sided collision. Only collides on the right side.
// Several of these are generated for a chain shape.
// ghost1 -> point1 -> point2 -> ghost2
@[typedef] 
struct C.b2SmoothSegment {
	// The tail ghost vertex
	ghost1 Vec2

	// The line segment
	segment Segment 

	// The head ghost vertex
	ghost2 Vec2 

	// The owning chain shape index (internal usage only)
	chainId int
}

pub type SmoothSegment = C.b2SmoothSegment

// A convex hull. Used to create convex polygons.
@[typedef] 
struct C.b2Hull {
	// The final points of the hull
	points [max_polygon_vertices]Vec2

	// The number of points
	count int
} 

pub type Hull = C.b2Hull

// Result of computing the distance between two line segments
@[typedef] 
struct C.b2SegmentDistanceResult {
	// The closest point on the first segment
	closest1 Vec2 

	// The closest point on the second segment
	closest2 Vec2

	// The barycentric coordinate on the first segment
	fraction1 f32

	// The barycentric coordinate on the second segment
	fraction2 f32

	// The squared distance between the closest points
	distanceSquared f32
}

pub type SegmentDistanceResult = C.b2SegmentDistanceResult

// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
@[typedef] 
struct C.b2DistanceProxy {
	// The point cloud
	points [max_polygon_vertices]Vec2

	// The number of points
	count int

	// The external radius of the point cloud
	radius f32
}

pub type DistanceProxy = C.b2DistanceProxy

// Used to warm start b2Distance. Set count to zero on first call or
//	use zero initialization.
@[typedef] 
struct C.b2DistanceCache {
	
	// The number of stored simplex points
	count u16

	// The cached simplex indices on shape A
	indexA [3]u8

	// The cached simplex indices on shape B
	indexB [3]u8
}

pub type DistanceCache = C.b2DistanceCache

pub const empty_distance_cache = DistanceCache{}

// Input for b2ShapeDistance
@[typedef] 
struct C.b2DistanceInput {
	// The proxy for shape A
	proxyA DistanceProxy 

	// The proxy for shape B
	proxyB DistanceProxy

	// The world transform for shape A
	transformA Transform 

	// The world transform for shape B
	transformB Transform

	// Should the proxy radius be considered?
	useRadii bool
}

pub type DistanceInput = C.b2DistanceInput

// Output for b2ShapeDistance
@[typedef] 
struct C.b2DistanceOutput {
	pointA Vec2  //< Closest point on shapeA
	pointB Vec2  //< Closest point on shapeB
	distance f32	//< The final distance, zero if overlapped
	iterations int //< Number of GJK iterations used
	simplexCount int //< The number of simplexes stored in the simplex array
}

pub type DistanceOutput = C.b2DistanceOutput

// Simplex vertex for debugging the GJK algorithm
@[typedef] 
struct C.b2SimplexVertex {
	wA Vec2 	//< support point in proxyA
	wB Vec2 	//< support point in proxyB
	w Vec2	//< wB - wA
	a f32	//< barycentric coordinate for closest point
	indexA int //< wA index
	indexB int //< wB index
}

pub type SimplexVertex = C.b2SimplexVertex

// Simplex from the GJK algorithm
@[typedef] 
struct C.b2Simplex {
	// vertices
	v1 SimplexVertex
	v2 SimplexVertex
	v3 SimplexVertex

	count int //< number of valid vertices
}

pub type Simplex = C.b2Simplex

// Input parameters for b2ShapeCast
@[typedef] 
struct C.b2ShapeCastPairInput {
	proxyA DistanceProxy  //< The proxy for shape A
	proxyB DistanceProxy //< The proxy for shape B
	transformA Transform  //< The world transform for shape A
	transformB Transform //< The world transform for shape B
	translationB Vec2  //< The translation of shape B
	maxFraction f32 //< The fraction of the translation to consider, typically 1
}

pub type ShapeCastPairInput = C.b2ShapeCastPairInput

// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
// position.
@[typedef] 
struct C.b2Sweep {
	localCenter Vec2 //< Local center of mass position
	c1 Vec2 //< Starting center of mass world position
	c2 Vec2 //< Ending center of mass world position
	q1 Rot  //< Starting world rotation
	q2 Rot //< Ending world rotation
}

pub type Sweep = C.b2Sweep

// Input parameters for b2TimeOfImpact
@[typedef] 
struct C.b2TOIInput {
	proxyA DistanceProxy  //< The proxy for shape A
	proxyB DistanceProxy //< The proxy for shape B
	sweepA Sweep  //< The movement of shape A
	sweepB Sweep //< The movement of shape B
	tMax f32 //< Defines the sweep interval [0, tMax]
}

pub type TOIInput = C.b2TOIInput

// Describes the TOI output
pub enum TOIState {
	to_i_state_unknown = 0
	to_i_state_failed = 1
	to_i_state_overlapped = 2
	to_i_state_hit = 3
	to_i_state_separated = 4
}

// Output parameters for b2TimeOfImpact.
@[typedef] 
struct C.b2TOIOutput {
	state TOIState  //< The type of result
	t f32 //< The time of the collision
}

pub type TOIOutput = C.b2TOIOutput

// A manifold point is a contact point belonging to a contact
// manifold. It holds details related to the geometry and dynamics
// of the contact points.
@[typedef] 
struct C.b2ManifoldPoint {
	// Location of the contact point in world space. Subject to precision loss at large coordinates.
	//	@note Should only be used for debugging.
	point Vec2 

	// Location of the contact point relative to bodyA's origin in world space
	//	@note When used internally to the Box2D solver, these are relative to the center of mass.
	anchorA Vec2

	// Location of the contact point relative to bodyB's origin in world space
	anchorB Vec2

	// The separation of the contact point, negative if penetrating
	separation f32

	// The impulse along the manifold normal vector.
	normalImpulse f32

	// The friction impulse
	tangentImpulse f32

	// The maximum normal impulse applied during sub-stepping
	//	todo not sure this is needed
	maxNormalImpulse f32

	// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
	// zero then there was no hit. Negative means shapes are approaching.
	normalVelocity f32

	// Uniquely identifies a contact point between two shapes
	id u16

	// Did this contact point exist the previous step?
	persisted bool
}

pub type ManifoldPoint = C.b2ManifoldPoint

// A contact manifold describes the contact points between colliding shapes
@[typedef] 
struct C.b2Manifold {
	// The manifold points, up to two are possible in 2D
	points [2]ManifoldPoint

	// The unit normal vector in world space, points from shape A to bodyB
	normal Vec2 

	// The number of contacts points, will be 0, 1, or 2
	pointCount int
}

pub type Manifold = C.b2Manifold

pub const (
	// The default category bit for a tree proxy. Used for collision filtering.
	default_category_bits = u32(0x00000001)

	// Convenience mask bits to use when you don't need collision filtering and just want
	//	all results.
	default_mask_bits = u32(0xFFFFFFFF)
)

// A node in the dynamic tree. This is private data placed here for performance reasons.
// 16 + 16 + 8 + pad(8)
@[typedef] 
struct C.b2TreeNode {
	// The node bounding box
	aabb AABB  // 16

	// Category bits for collision filtering
	categoryBits u32 // 4

	// The node parent index
	parent int

	// The node freelist next index
	next int

	// Child 1 index
	child1 int  // 4

	// Child 2 index
	child2 int // 4

	// User data 
	// todo could be union with child index
	userData int // 4

	// Leaf = 0, free node = -1
	height i16 // 2

	// Has the AABB been enlarged?
	enlarged bool // 1

	// Padding for clarity
	pad [9]char
}

pub type TreeNode = C.b2TreeNode

// The dynamic tree structure. This should be considered private data.
// It is placed here for performance reasons.
@[typedef] 
struct C.b2DynamicTree {
	// The tree nodes
	nodes &TreeNode

	// The root index
	root int

	// The number of nodes
	nodeCount int

	// The allocated node space
	nodeCapacity int

	// Node free list
	freeList int

	// Number of proxies created
	proxyCount int

	// Leaf indices for rebuild
	leafIndices &int

	// Leaf bounding boxes for rebuild
	leafBoxes &AABB 

	// Leaf bounding box centers for rebuild
	leafCenters &Vec2 

	// Bins for sorting during rebuild
	binIndices &int

	// Allocated space for rebuilding
	rebuildCapacity int
}

pub type DynamicTree = C.b2DynamicTree


pub type TreeQueryCallbackFcn = fn(proxyId int, userData int, context voidptr) bool
pub type TreeRayCastCallbackFcn = fn(input &RayCastInput, proxyId int, userData int, context voidptr) f32
pub type TreeShapeCastCallbackFcn = fn(input &ShapeCastInput, proxyId int, userData int, context voidptr) f32


// Validate ray cast input data (NaN, etc)
fn C.b2IsValidRay(input &RayCastInput) bool
@[inline]
pub fn is_valid_ray(input &RayCastInput) bool {
	return C.b2IsValidRay(input)
}

// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
fn C.b2MakePolygon(hull &Hull, radius f32) Polygon
@[inline]
pub fn make_polygon(hull &Hull, radius f32) Polygon {
	return C.b2MakePolygon(hull, radius)
}

// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
fn C.b2MakeOffsetPolygon(hull &Hull, radius f32, transform Transform) Polygon
@[inline]
pub fn make_offset_polygon(hull &Hull, radius f32, transform Transform) Polygon {
	return C.b2MakePolygon(hull, radius)
}

// Make a square polygon, bypassing the need for a convex hull.
fn C.b2MakeSquare(h f32) Polygon
@[inline]
pub fn make_square(h f32) Polygon {
	return C.b2MakeSquare(h)
}

// Make a box (rectangle) polygon, bypassing the need for a convex hull.
fn C.b2MakeBox(hx f32, hy f32) Polygon
@[inline]
pub fn make_box(hx f32, hy f32) Polygon {
	return C.b2MakeBox(hx, hy)
}

// Make a rounded box, bypassing the need for a convex hull.
fn C.b2MakeRoundedBox(hx f32, hy f32, radius f32) Polygon
@[inline]
pub fn make_rounded_box(hx f32, hy f32, radius f32) Polygon {
	return C.b2MakeRoundedBox(hx, hy, radius)
}

// Make an offset box, bypassing the need for a convex hull.
fn C.b2MakeOffsetBox(hx f32, hy f32, center Vec2, angle f32) Polygon
@[inline]
pub fn make_offset_box(hx f32, hy f32, center Vec2, angle f32) Polygon {
	return C.b2MakeOffsetBox(hx, hy, center, angle)
}

// Transform a polygon. This is useful for transferring a shape from one body to another.
fn C.b2TransformPolygon(transform Transform, polygon &Polygon) Polygon
@[inline]
pub fn transform_polygon(transform Transform, polygon &Polygon) Polygon {
	return C.b2TransformPolygon(transform, polygon)
}

// Compute mass properties of a circle
fn C.b2ComputeCircleMass(shape &Circle, density f32) MassData
@[inline]
pub fn compute_circle_mass(shape &Circle, density f32) MassData {
	return C.b2ComputeCircleMass(shape, density)
}

// Compute mass properties of a capsule
fn C.b2ComputeCapsuleMass(shape &Capsule, density f32) MassData
@[inline]
pub fn compute_capsule_mass(shape &Capsule, density f32) MassData {
	return C.b2ComputeCapsuleMass(shape, density)
}

// Compute mass properties of a polygon
fn C.b2ComputePolygonMass(shape &Polygon, density f32) MassData
@[inline]
pub fn compute_polgon_mass(shape &Polygon, density f32) MassData {
	return C.b2ComputePolygonMass(shape, density)
}

// Compute the bounding box of a transformed circle
fn C.b2ComputeCircleAABB(shape &Circle, transform Transform) AABB
@[inline]
pub fn compute_circle_aabb(shape &Circle, transform Transform) AABB {
	return C.b2ComputeCircleAABB(shape, transform)
}

// Compute the bounding box of a transformed capsule
fn C.b2ComputeCapsuleAABB(shape &Capsule, transform Transform) AABB
@[inline]
pub fn compute_capsule_aabb(shape &Capsule, transform Transform) AABB {
	return C.b2ComputeCapsuleAABB(shape, transform)
}

// Compute the bounding box of a transformed polygon
fn C.b2ComputePolygonAABB(shape &Polygon, transform Transform) AABB
@[inline]
pub fn compute_polygon_aabb(shape &Polygon, transform Transform) AABB {
	return C.b2ComputePolygonAABB(shape, transform)
}

// Compute the bounding box of a transformed line segment
fn C.b2ComputeSegmentAABB(shape &Segment, transform Transform) AABB
@[inline]
pub fn compute_segment_aabb(shape &Segment, transform Transform) AABB {
	return C.b2ComputeSegmentAABB(shape, transform)
}

// Test a point for overlap with a circle in local space
fn C.b2PointInCircle(point Vec2, shape &Circle) bool
@[inline]
pub fn point_in_circle(point Vec2, shape &Circle) bool {
	return C.b2PointInCircle(point, shape)
}

// Test a point for overlap with a circle in local space
fn C.b2PointInCapsule(point Vec2, shape &Capsule) bool
@[inline]
pub fn point_in_capsule(point Vec2, shape &Capsule) bool {
	return C.b2PointInCapsule(point, shape)
}

// Test a point for overlap with a convex polygon in local space
fn C.b2PointInPolygon(point Vec2, shape &Polygon) bool
@[inline]
pub fn point_in_polygon(point Vec2, shape &Polygon) bool {
	return C.b2PointInPolygon(point, shape)
}

// Ray cast versus circle in shape local space. Initial overlap is treated as a miss.
fn C.b2RayCastCircle(input &RayCastInput, shape &Circle) CastOutput
@[inline]
pub fn ray_cast_circle(input &RayCastInput, shape &Circle) CastOutput {
	return C.b2RayCastCircle(input, shape)
}

// Ray cast versus capsule in shape local space. Initial overlap is treated as a miss.
fn C.b2RayCastCapsule(input &RayCastInput, shape &Capsule) CastOutput
@[inline]
pub fn ray_cast_capsule(input &RayCastInput, shape &Capsule) CastOutput {
	return C.b2RayCastCapsule(input, shape)
}

// Ray cast versus segment in shape local space. Optionally treat the segment as one-sided with hits from
// the left side being treated as a miss.
fn C.b2RayCastSegment(input &RayCastInput, shape &Segment, oneSided bool) CastOutput
@[inline]
pub fn ray_cast_segment(input &RayCastInput, shape &Segment, oneSided bool) CastOutput {
	return C.b2RayCastSegment(input, shape, oneSided)
}

// Ray cast versus polygon in shape local space. Initial overlap is treated as a miss.
fn C.b2RayCastPolygon(input &RayCastInput, shape &Polygon) CastOutput
@[inline]
pub fn ray_cast_polygon(input &RayCastInput, shape &Polygon) CastOutput {
	return C.b2RayCastPolygon(input, shape)
}


// Shape cast versus a circle. Initial overlap is treated as a miss.
fn C.b2ShapeCastCircle(input &ShapeCastInput, shape &Circle) CastOutput
@[inline]
pub fn shape_cast_circle(input &ShapeCastInput, shape &Circle) CastOutput {
	return C.b2ShapeCastCircle(input, shape)
}

// Shape cast versus a capsule. Initial overlap is treated as a miss.
fn C.b2ShapeCastCapsule(input &ShapeCastInput, shape &Capsule) CastOutput
@[inline]
pub fn shape_cast_capsule(input &ShapeCastInput, shape &Capsule) CastOutput {
	return C.b2ShapeCastCapsule(input, shape)
}

// Shape cast versus a line segment. Initial overlap is treated as a miss.
fn C.b2ShapeCastSegment(input &ShapeCastInput, shape &Segment) CastOutput
@[inline]
pub fn shape_cast_segment(input &ShapeCastInput, shape &Segment) CastOutput {
	return C.b2ShapeCastSegment(input, shape)
}

// Shape cast versus a convex polygon. Initial overlap is treated as a miss.
fn C.b2ShapeCastPolygon(input &ShapeCastInput, shape &Polygon) CastOutput
@[inline]
pub fn shape_cast_polygon(input &ShapeCastInput, shape &Polygon) CastOutput {
	return C.b2ShapeCastPolygon(input, shape)
}

// Test a point for overlap with a circle in local space
fn C.b2ComputeHull(points &Vec2, count int) Hull
@[inline]
pub fn compute_hull(points &Vec2, count int) Hull {
	return C.b2ComputeHull(points, count)
}

// This determines if a hull is valid. Checks for:
// - convexity
// - collinear points
// This is expensive and should not be called at runtime.
fn C.b2ValidateHull(hull &Hull) bool
@[inline]
pub fn validate_hull(hull &Hull) bool {
	return C.b2ValidateHull(hull)
}

// Compute the distance between two line segments, clamping at the end points if needed.
fn C.b2SegmentDistance(p1 Vec2, q1 Vec2, p2 Vec2, q2 Vec2) SegmentDistanceResult 
@[inline]
pub fn segment_distance(p1 Vec2, q1 Vec2, p2 Vec2, q2 Vec2) SegmentDistanceResult {
	return C.b2SegmentDistance(p1, q1, p2, q2)
}

// Compute the closest points between two shapes represented as point clouds.
// b2DistanceCache cache is input/output. On the first call set b2DistanceCache.count to zero.
//	The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
fn C.b2ShapeDistance(cache &DistanceCache, input &DistanceInput, simplexes &Simplex, simplexCapacity int)  DistanceOutput 
@[inline]
pub fn shape_distance(cache &DistanceCache, input &DistanceInput, simplexes &Simplex, simplexCapacity int)  DistanceOutput {
	return C.b2ShapeDistance(cache, input, simplexes, simplexCapacity)
}

// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
fn C.b2ShapeCast(input &ShapeCastPairInput) CastOutput 
@[inline]
pub fn shape_cast(input &ShapeCastPairInput) CastOutput  {
	return C.b2ShapeCast(input)
}

// Make a proxy for use in GJK and related functions.
fn C.b2MakeProxy(vertices &Vec2, count int, radius f32) DistanceProxy
@[inline]
pub fn make_proxy(vertices &Vec2, count int, radius f32) DistanceProxy  {
	return C.b2MakeProxy(vertices, count, radius)
}

// Evaluate the transform sweep at a specific time.
fn C.b2GetSweepTransform(sweep &Sweep, time f32) Transform 
@[inline]
pub fn get_sweep_transform(sweep &Sweep, time f32) Transform  {
	return C.b2GetSweepTransform(sweep, time)
}

// Compute the upper bound on time before two shapes penetrate. Time is represented as
// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
// non-tunneling collisions. If you change the time interval, you should call this function
// again.
fn C.b2TimeOfImpact(input &TOIInput) TOIOutput  
@[inline]
pub fn time_of_impact(input &TOIInput) TOIOutput  {
	return C.b2TimeOfImpact(input)
}

// Compute the contact manifold between two circles
fn C.b2CollideCircles(circleA &Circle, xfA Transform, circleB &Circle, xfB Transform) Manifold 
@[inline]
pub fn collide_circles(circleA &Circle, xfA Transform, circleB &Circle, xfB Transform) Manifold  {
	return C.b2CollideCircles(circleA, xfA, circleB, xfB)
}

// Compute the contact manifold between a capsule and circle
fn C.b2CollideCapsuleAndCircle(capsuleA &Capsule, xfA Transform, circleB &Circle, xfB Transform) Manifold 
@[inline]
pub fn collide_capsule_and_circle(capsuleA &Capsule, xfA Transform, circleB &Circle, xfB Transform) Manifold  {
	return C.b2CollideCapsuleAndCircle(capsuleA, xfA, circleB, xfB)
}

// Compute the contact manifold between an segment and a circle
fn C.b2CollideSegmentAndCircle(segmentA &Segment, xfA Transform, circleB &Circle, xfB Transform) Manifold 
@[inline]
pub fn collide_segment_and_circle(segmentA &Segment, xfA Transform, circleB &Circle, xfB Transform) Manifold  {
	return C.b2CollideSegmentAndCircle(segmentA, xfA, circleB, xfB)
}

// Compute the contact manifold between a polygon and a circle
fn C.b2CollidePolygonAndCircle(polygonA &Polygon, xfA Transform, circleB &Circle, xfB Transform) Manifold 
@[inline]
pub fn collide_polygon_and_circle(polygonA &Polygon, xfA Transform, circleB &Circle, xfB Transform) Manifold  {
	return C.b2CollidePolygonAndCircle(polygonA, xfA, circleB, xfB)
}

// Compute the contact manifold between a capsule and circle
fn C.b2CollideCapsules(capsuleA &Capsule, xfA Transform, capsuleB &Capsule, xfB Transform) Manifold 
@[inline]
pub fn collide_capsules(capsuleA &Capsule, xfA Transform, capsuleB &Capsule, xfB Transform) Manifold  {
	return C.b2CollideCapsules(capsuleA, xfA, capsuleB, xfB)
}

// Compute the contact manifold between an segment and a capsule
fn C.b2CollideSegmentAndCapsule(segmentA &Segment, xfA Transform, capsuleB &Capsule, xfB Transform) Manifold 
@[inline]
pub fn collide_segment_and_capsule(segmentA &Segment, xfA Transform, capsuleB &Capsule, xfB Transform) Manifold  {
	return C.b2CollideSegmentAndCapsule(segmentA, xfA, capsuleB, xfB)
}

// Compute the contact manifold between a polygon and capsule
fn C.b2CollidePolygonAndCapsule(polygonA &Polygon, xfA Transform, capsuleB &Capsule, xfB Transform) Manifold 
@[inline]
pub fn collide_polygon_and_capsule(polygonA &Polygon, xfA Transform, capsuleB &Capsule, xfB Transform) Manifold  {
	return C.b2CollidePolygonAndCapsule(polygonA, xfA, capsuleB, xfB)
}

// Compute the contact manifold between two polygons
fn C.b2CollidePolygons(polygonA &Polygon, xfA Transform, polygonB &Polygon, xfB Transform) Manifold 
@[inline]
pub fn collide_polygons(polygonA &Polygon, xfA Transform, polygonB &Polygon, xfB Transform) Manifold  {
	return C.b2CollidePolygons(polygonA, xfA, polygonB, xfB)
}

// Compute the contact manifold between an segment and a polygon
fn C.b2CollideSegmentAndPolygon(segmentA &Segment, xfA Transform, polygonB &Polygon, xfB Transform) Manifold 
@[inline]
pub fn collide_segment_and_polygon(segmentA &Segment, xfA Transform, polygonB &Polygon, xfB Transform) Manifold  {
	return C.b2CollideSegmentAndPolygon(segmentA, xfA, polygonB, xfB)
}

// Compute the contact manifold between a smooth segment and a circle
fn C.b2CollideSmoothSegmentAndCircle(smoothSegmentA &SmoothSegment, xfA Transform, circleB &Circle, xfB Transform) Manifold
@[inline]
pub fn collide_smooth_segment_and_circle(smoothSegmentA &SmoothSegment, xfA Transform, circleB &Circle, xfB Transform) Manifold  {
	return C.b2CollideSmoothSegmentAndCircle(smoothSegmentA, xfA, circleB, xfB)
}

// Compute the contact manifold between an segment and a capsule
fn C.b2CollideSmoothSegmentAndCapsule(smoothSegmentA &SmoothSegment, xfA Transform, capsuleB &Capsule, xfB Transform, cache &DistanceCache) Manifold 
@[inline]
pub fn collide_smooth_segment_and_capsule(smoothSegmentA &SmoothSegment, xfA Transform, capsuleB &Capsule, xfB Transform, cache &DistanceCache) Manifold  {
	return C.b2CollideSmoothSegmentAndCapsule(smoothSegmentA, xfA, capsuleB, xfB, cache)
}

// Compute the contact manifold between a smooth segment and a rounded polygon
fn C.b2CollideSmoothSegmentAndPolygon(smoothSegmentA &SmoothSegment, xfA Transform, polygonB &Polygon, xfB Transform, cache &DistanceCache) Manifold 
@[inline]
pub fn collide_smooth_segment_and_polygon(smoothSegmentA &SmoothSegment, xfA Transform, polygonB &Polygon, xfB Transform, cache &DistanceCache) Manifold  {
	return C.b2CollideSmoothSegmentAndPolygon(smoothSegmentA, xfA, polygonB, xfB, cache)
}

// Constructing the tree initializes the node pool.
fn C.b2DynamicTree_Create() DynamicTree
@[inline]
pub fn dynamic_tree_create() DynamicTree {
	return C.b2DynamicTree_Create()
}

// Destroy the tree, freeing the node pool.
fn C.b2DynamicTree_Destroy(tree &DynamicTree)
@[inline]
pub fn dynamic_tree_destroy(tree &DynamicTree) {
	C.b2DynamicTree_Destroy(tree)
}

// Create a proxy. Provide an AABB and a userData value.
fn C.b2DynamicTree_CreateProxy(tree &DynamicTree, aabb AABB, categoryBits u32, userData int) int
@[inline]
pub fn dynamic_tree_create_proxy(tree &DynamicTree, aabb AABB, categoryBits u32, userData int) int {
	return C.b2DynamicTree_CreateProxy(tree, aabb, categoryBits, userData)
}

// Destroy a proxy. This asserts if the id is invalid.
fn C.b2DynamicTree_DestroyProxy(tree &DynamicTree, proxyId int)
@[inline]
pub fn dynamic_tree_destroy_proxy(tree &DynamicTree, proxyId int) {
	C.b2DynamicTree_DestroyProxy(tree, proxyId)
}

// Move a proxy to a new AABB by removing and reinserting into the tree.
fn C.b2DynamicTree_MoveProxy(tree &DynamicTree, proxyId int, aabb AABB)
@[inline]
pub fn dynamic_tree_move_proxy(tree &DynamicTree, proxyId int, aabb AABB) {
	C.b2DynamicTree_MoveProxy(tree, proxyId, aabb)
}

// Enlarge a proxy and enlarge ancestors as necessary.
fn C.b2DynamicTree_EnlargeProxy(tree &DynamicTree, proxyId int, aabb AABB)
@[inline]
pub fn dynamic_tree_enlarge_proxy(tree &DynamicTree, proxyId int, aabb AABB) {
	C.b2DynamicTree_EnlargeProxy(tree, proxyId, aabb)
}

// Query an AABB for overlapping proxies. The callback class
// is called for each proxy that overlaps the supplied AABB.
fn C.b2DynamicTree_Query(tree &DynamicTree, aabb AABB, maskBits u32, callback &TreeQueryCallbackFcn, context voidptr)
@[inline]
pub fn dynamic_tree_query(tree &DynamicTree, aabb AABB, maskBits u32, callback &TreeQueryCallbackFcn, context voidptr) {
	C.b2DynamicTree_Query(tree, aabb, maskBits, callback, context)
}

// Ray-cast against the proxies in the tree. This relies on the callback
// to perform a exact ray-cast in the case were the proxy contains a shape.
// The callback also performs the any collision filtering. This has performance
// roughly equal to k * log(n), where k is the number of collisions and n is the
// number of proxies in the tree.
//	Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
//	@param tree the dynamic tree to ray cast
// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)
//	@param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
// @param callback a callback class that is called for each proxy that is hit by the ray
//	@param context user context that is passed to the callback
fn C.b2DynamicTree_RayCast(tree &DynamicTree, input &RayCastInput, maskBits u32, callback &TreeRayCastCallbackFcn, context voidptr)
@[inline]
pub fn dynamic_tree_ray_cast(tree &DynamicTree, input &RayCastInput, maskBits u32, callback &TreeRayCastCallbackFcn, context voidptr) {
	C.b2DynamicTree_RayCast(tree, input, maskBits, callback, context)
}

// Ray-cast against the proxies in the tree. This relies on the callback
// to perform a exact ray-cast in the case were the proxy contains a shape.
// The callback also performs the any collision filtering. This has performance
// roughly equal to k * log(n), where k is the number of collisions and n is the
// number of proxies in the tree.
//	@param tree the dynamic tree to ray cast
// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
//	@param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
// @param callback a callback class that is called for each proxy that is hit by the shape
//	@param context user context that is passed to the callback
fn C.b2DynamicTree_ShapeCast(tree &DynamicTree, input &ShapeCastInput, maskBits u32, callback &TreeShapeCastCallbackFcn, context voidptr)
@[inline]
pub fn dynamic_tree_shape_cast(tree &DynamicTree, input &ShapeCastInput, maskBits u32, callback &TreeShapeCastCallbackFcn, context voidptr) {
	C.b2DynamicTree_ShapeCast(tree, input, maskBits, callback, context)
}

// Validate this tree. For testing.
fn C.b2DynamicTree_Validate(tree &DynamicTree)
@[inline]
pub fn dynamic_tree_validate(tree &DynamicTree) {
	C.b2DynamicTree_Validate(tree)
}

// Compute the height of the binary tree in O(N) time. Should not be
// called often.
fn C.b2DynamicTree_GetHeight(tree &DynamicTree) int
@[inline]
pub fn dynamic_tree_get_height(tree &DynamicTree) int {
	return C.b2DynamicTree_GetHeight(tree)
}

// Get the maximum balance of the tree. The balance is the difference in height of the two children of a node.
fn C.b2DynamicTree_GetMaxBalance(tree &DynamicTree) int
@[inline]
pub fn dynamic_tree_get_max_balance(tree &DynamicTree) int {
	return C.b2DynamicTree_GetMaxBalance(tree)
}

// Get the ratio of the sum of the node areas to the root area.
fn C.b2DynamicTree_GetAreaRatio(tree &DynamicTree) f32
@[inline]
pub fn dynamic_tree_get_area_ratio(tree &DynamicTree) f32 {
	return C.b2DynamicTree_GetAreaRatio(tree)
}

// Build an optimal tree. Very expensive. For testing.
fn C.b2DynamicTree_RebuildBottomUp(tree &DynamicTree)
@[inline]
pub fn dynamic_tree_rebuild_bottom_up(tree &DynamicTree) {
	C.b2DynamicTree_RebuildBottomUp(tree)
}

// Get the number of proxies created
fn C.b2DynamicTree_GetProxyCount(tree &DynamicTree) int
@[inline]
pub fn dynamic_tree_get_proxy_count(tree &DynamicTree) int {
	return C.b2DynamicTree_GetProxyCount(tree)
}

// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
fn C.b2DynamicTree_Rebuild(tree &DynamicTree, fullBuild bool) int
@[inline]
pub fn dynamic_tree_rebuild(tree &DynamicTree, fullBuild bool) int {
	return C.b2DynamicTree_Rebuild(tree, fullBuild)
}

// Shift the world origin. Useful for large worlds.
// The shift formula is: position -= newOrigin
// @param tree the tree to shift
// @param newOrigin the new origin with respect to the old origin
fn C.b2DynamicTree_ShiftOrigin(tree &DynamicTree, newOrigin Vec2)
@[inline]
pub fn dynamic_tree_shift_origin(tree &DynamicTree, newOrigin Vec2) {
	C.b2DynamicTree_ShiftOrigin(tree, newOrigin)
}

// Get the number of bytes used by this tree 
fn C.b2DynamicTree_GetByteCount(tree &DynamicTree) int
@[inline]
pub fn dynamic_tree_get_byte_count(tree &DynamicTree) int {
	return C.b2DynamicTree_GetByteCount(tree)
}

// Get proxy user data
// @return the proxy user data or 0 if the id is invalid
fn C.b2DynamicTree_GetUserData(tree &DynamicTree, proxyId int) int
@[inline]
pub fn dynamic_tree_get_user_data(tree &DynamicTree, proxyId int) int {
	return C.b2DynamicTree_GetUserData(tree, proxyId)
}

// Get the AABB of a proxy
fn C.b2DynamicTree_GetAABB(tree &DynamicTree , proxyId int) AABB
@[inline]
pub fn dynamic_tree_get_aabb(tree &DynamicTree , proxyId int) AABB {
	return C.b2DynamicTree_GetAABB(tree, proxyId)
}

// end collision

// start types 

type TaskCallback = fn(startIndex int, endIndex int, workerIndex u32, taskContext voidptr) 
type EnqueueTaskCallback = fn(task &TaskCallback, itemCount int, minRange int, taskContext voidptr, userContext voidptr) voidptr
type FinishTaskCallback = fn(userTask voidptr, userContext voidptr)

// Result from b2World_RayCastClosest
// ingroup world
@[typedef]
struct C.b2RayResult {
pub mut:
	shapeId ShapeId 
	point Vec2 
	normal Vec2 
	fraction f32
	hit bool
}

pub type RayResult = C.b2RayResult

// World definition used to create a simulation world.
// Must be initialized using b2DefaultWorldDef().
// ingroup world
@[typedef] 
struct C.b2WorldDef {
pub mut:
	// Gravity vector. Box2D has no up-vector defined.
	gravity Vec2 

	// Restitution velocity threshold, usually in m/s. Collisions above this
	// speed have restitution applied (will bounce).
	restitutionThreshold f32

	// This parameter controls how fast overlap is resolved and has units of meters per second
	contactPushoutVelocity f32

	// Threshold velocity for hit events. Usually meters per second.
	hitEventThreshold f32

	// Contact stiffness. Cycles per second.
	contactHertz f32

	// Contact bounciness. Non-dimensional.
	contactDampingRatio f32

	// Joint stiffness. Cycles per second.
	jointHertz f32

	// Joint bounciness. Non-dimensional.
	jointDampingRatio f32

	// Can bodies go to sleep to improve performance
	enableSleep bool

	// Enable continuous collision
	enableContinous bool

	// Number of workers to use with the provided task system. Box2D performs best when using only
	//	performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
	//	little benefit and may even harm performance.
	workerCount int

	// Function to spawn tasks
	enqueueTask &EnqueueTaskCallback 

	// Function to finish a task
	finishTask &FinishTaskCallback 

	// User context that is provided to enqueueTask and finishTask
	userTaskContext voidptr

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type WorldDef = C.b2WorldDef

// The body simulation type.
// Each body is one of these three types. The type determines how the body behaves in the simulation.
// @ingroup body
pub enum BodyType {
	// zero mass, zero velocity, may be manually moved
	static_body = 0

	// zero mass, velocity set by user, moved by solver
	kinematic_body = 1

	// positive mass, velocity determined by forces, moved by solver
	dynamic_body = 2

	// number of body types
	body_type_count = 3
}

// A body definition holds all the data needed to construct a rigid body.
// You can safely re-use body definitions. Shapes are added to a body after construction.
//	Body definitions are temporary objects used to bundle creation parameters.
// Must be initialized using b2DefaultBodyDef().
// @ingroup body
@[typedef] 
struct C.b2BodyDef {
pub mut:
	// The body type: static, kinematic, or dynamic.
	types BodyType 

	// The initial world position of the body. Bodies should be created with the desired position.
	// @note Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
	//	if the body is moved after shapes have been added.
	position Vec2 

	// The initial world angle of the body in radians.
	rotation Rot

	// The initial linear velocity of the body's origin. Typically in meters per second.
	linearVelocity Vec2

	// The initial angular velocity of the body. Radians per second.
	angularVelocity f32

	// Linear damping is use to reduce the linear velocity. The damping parameter
	// can be larger than 1 but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	//	Generally linear damping is undesirable because it makes objects move slowly
	//	as if they are floating.
	linearDamping f32

	// Angular damping is use to reduce the angular velocity. The damping parameter
	// can be larger than 1.0f but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	//	Angular damping can be use slow down rotating bodies.
	angularDamping f32

	// Scale the gravity applied to this body. Non-dimensional.
	gravityScale f32

	// Sleep velocity threshold, default is 0.05 meter per second
	sleepThreshold f32

	// Use this to store application specific body data.
	userData voidptr

	// Set this flag to false if this body should never fall asleep.
	enableSleep bool

	// Is this body initially awake or sleeping?
	isAwake bool

	// Should this body be prevented from rotating? Useful for characters.
	fixedRotation bool

	// Treat this body as high speed object that performs continuous collision detection
	// against dynamic and kinematic bodies, but not other bullet bodies.
	//	@warning Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
	//	continuous collision. They may interfere with joint constraints.
	isBullet bool

	// Used to disable a body. A disabled body does not move or collide.
	isEnabled bool

	// Automatically compute mass and related properties on this body from shapes.
	// Triggers whenever a shape is add/removed/changed. Default is true.
	automaticMass bool

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type BodyDef = C.b2BodyDef

// This is used to filter collision on shapes. It affects shape-vs-shape collision
//	and shape-versus-query collision (such as b2World_CastRay).
// @ingroup shape
@[typedef] 
struct C.b2Filter {
pub mut:
	// The collision category bits. Normally you would just set one bit. The category bits should
	//	represent your application object types. For example:
	//	@code{.cpp}
	//	enum MyCategories
	//	{
	//	   Static  = 0x00000001,
	//	   Dynamic = 0x00000002,
	//	   Debris  = 0x00000004,
	//	   Player  = 0x00000008,
	//	   // etc
	// }
	//	@endcode
	categoryBits u32

	// The collision mask bits. This states the categories that this
	// shape would accept for collision.
	//	For example, you may want your player to only collide with static objects
	//	and other players.
	//	@code{.c}
	//	maskBits = Static | Player
	//	@endcode
	maskBits u32

	// Collision groups allow a certain group of objects to never collide (negative)
	// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
	// always wins against the mask bits.
	//	For example, you may want ragdolls to collide with other ragdolls but you don't want
	//	ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
	//	and apply that group index to all shapes on the ragdoll.
	groupIndex int
}

pub type Filter = C.b2Filter

// The query filter is used to filter collisions between queries and shapes. For example,
//	you may want a ray-cast representing a projectile to hit players and the static environment
//	but not debris.
// @ingroup shape
@[typedef] 
struct C.b2QueryFilter {
	// The collision category bits of this query. Normally you would just set one bit.
	categoryBits u32

	// The collision mask bits. This states the shape categories that this
	// query would accept for collision.
	maskBits u32
}

pub type QueryFilter = C.b2QueryFilter

// Shape type
// @ingroup shape
pub enum ShapeType {
	// A circle with an offset
	circle_shape

	// A capsule is an extruded circle
	capsule_shape

	// A line segment
	segment_shape

	// A convex polygon
	polygon_shape

	// A smooth segment owned by a chain shape
	smooth_segment_shape

	// The number of shape types
	shape_type_count
}

// Used to create a shape.
// This is a temporary object used to bundle shape creation parameters. You may use
//	the same shape definition to create multiple shapes.
// Must be initialized using b2DefaultShapeDef().
// @ingroup shape
@[typedef] 
struct C.b2ShapeDef {
pub mut:
	// Use this to store application specific shape data.
	userData voidptr

	// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	friction f32

	// The restitution (bounce) usually in the range [0,1].
	restitution f32

	// The density, usually in kg/m^2.
	density f32

	// Collision filtering data.
	filter Filter

	// A sensor shape generates overlap events but never generates a collision response.
	isSensor bool

	// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enableSensorEvents bool

	// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enableContactEvents bool

	// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	enableHitEvents bool

	// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	//	and must be carefully handled due to threading. Ignored for sensors.
	enablePreSolveEvents bool

	// Normally shapes on static bodies don't invoke contact creation when they are added to the world. This overrides
	//	that behavior and causes contact creation. This significantly slows down static body creation which can be important
	//	when there are many static shapes.
	forceContactCreation bool

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type ShapeDef = C.b2ShapeDef

// Used to create a chain of edges. This is designed to eliminate ghost collisions with some limitations.
//	- chains are one-sided
//	- chains have no mass and should be used on static bodies
//	- chains have a counter-clockwise winding order
//	- chains are either a loop or open
// - a chain must have at least 4 points
//	- the distance between any two points must be greater than linearSlop
//	- a chain shape should not self intersect (this is not validated)
//	- an open chain shape has NO COLLISION on the first and final edge
//	- you may overlap two open chains on their first three and/or last three points to get smooth collision
//	- a chain shape creates multiple smooth edges shapes on the body
// https://en.wikipedia.org/wiki/Polygonal_chain
// Must be initialized using b2DefaultChainDef().
//	@warning Do not use chain shapes unless you understand the limitations. This is an advanced feature.
// @ingroup shape
@[typedef] 
struct C.b2ChainDef {
pub mut:
	// Use this to store application specific shape data.
	userData voidptr

	// An array of at least 4 points. These are cloned and may be temporary.
	points &Vec2

	// The point count, must be 4 or more.
	count int

	// The friction coefficient, usually in the range [0,1].
	friction f32

	// The restitution (elasticity) usually in the range [0,1].
	restitution f32

	// Contact filtering data.
	filter Filter 

	// Indicates a closed chain formed by connecting the first and last points
	isLoop bool

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type ChainDef = C.b2ChainDef

//! @cond
// Profiling data. Times are in milliseconds.
@[typedef] 
struct C.b2Profile {
pub mut:
	step f32
	pairs f32
	collide f32
	solve f32
	buildIslands f32
	solveConstraints f32
	prepareTasks f32
	solverTasks f32
	prepareConstraints f32
	integrateVelocities f32
	warmStart f32
	solveVelocities f32
	integratePositions f32
	relaxVelocities f32
	applyRestitution f32
	storeImpulses f32
	finalizeBodies f32
	splitIslands f32
	sleepIslands f32
	hitEvents f32
	broadphase f32
	continuous f32
}

pub type Profile = C.b2Profile

// Counters that give details of the simulation size.
@[typedef] 
struct C.b2Counters {
pub mut:
	staticBodyCount int
	bodyCount int
	shapeCount int
	contactCount int
	jointCount int
	islandCount int
	stackUsed int
	staticTreeHeight int
	treeHeight int
	byteCount int
	taskCount int
	colorCounts [12]int
}

pub type Counters = C.b2Counters

pub enum JointType {
	distance_joint
	motor_joint
	mouse_joint
	prismatic_joint
	revolute_joint
	weld_joint
	wheel_joint
} 

// Distance joint definition
//
// This requires defining an anchor point on both
// bodies and the non-zero distance of the distance joint. The definition uses
// local anchor points so that the initial configuration can violate the
// constraint slightly. This helps when saving and loading a game.
// @ingroup distance_joint
@[typedef] 
struct C.b2DistanceJointDef {
pub mut:	
	// The first attached body
	bodyIdA BodyId

	// The second attached body
	bodyIdB BodyId

	// The local anchor point relative to bodyA's origin
	localAnchorA Vec2 

	// The local anchor point relative to bodyB's origin
	localAnchorB Vec2

	// The rest length of this joint. Clamped to a stable minimum value.
	length f32

	// Enable the distance constraint to behave like a spring. If false
	//	then the distance joint will be rigid, overriding the limit and motor.
	enableSpring bool

	// The spring linear stiffness Hertz, cycles per second
	hertz f32

	// The spring linear damping ratio, non-dimensional
	dampingRatio f32

	// Enable/disable the joint limit
	enableLimit bool

	// Minimum length. Clamped to a stable minimum value.
	minLength f32

	// Maximum length. Must be greater than or equal to the minimum length.
	maxLength f32

	// Enable/disable the joint motor
	enableMotor bool

	// The maximum motor force, usually in newtons
	maxMotorForce f32

	// The desired motor speed, usually in meters per second
	motorSpeed f32

	// Set this flag to true if the attached bodies should collide
	collideConnected bool

	// User data pointer
	userData voidptr

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type DistanceJointDef = C.b2DistanceJointDef

// A motor joint is used to control the relative motion between two bodies
//
// A typical usage is to control the movement of a dynamic body with respect to the ground.
// @ingroup motor_joint
@[typedef] 
struct C.b2MotorJointDef {
pub mut:
	// The first attached body
	bodyIdA BodyId 

	// The second attached body
	bodyIdB BodyId

	// Position of bodyB minus the position of bodyA, in bodyA's frame
	linearOffset Vec2

	// The bodyB angle minus bodyA angle in radians
	angularOffset f32

	// The maximum motor force in newtons
	maxForce f32

	// The maximum motor torque in newton-meters
	maxTorque f32

	// Position correction factor in the range [0,1]
	correctionFactor f32

	// Set this flag to true if the attached bodies should collide
	collideConnected bool

	// User data pointer
	userData voidptr

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type MotorJointDef = C.b2MotorJointDef

// A mouse joint is used to make a point on a body track a specified world point.
//
// This a soft constraint and allows the constraint to stretch without
// applying huge forces. This also applies rotation constraint heuristic to improve control.
// @ingroup mouse_joint
@[typedef] 
struct C.b2MouseJointDef {
	// The first attached body.
	bodyIdA BodyId 

	// The second attached body.
	bodyIdB BodyId

	// The initial target point in world space
	target Vec2 

	// Stiffness in hertz
	hertz f32

	// Damping ratio, non-dimensional
	dampingRatio f32

	// Maximum force, typically in newtons
	maxForce f32

	// Set this flag to true if the attached bodies should collide.
	collideConnected bool

	// User data pointer
	userData voidptr

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type MouseJointDef = C.b2MouseJointDef

// Prismatic joint definition
//
// This requires defining a line of motion using an axis and an anchor point.
// The definition uses local anchor points and a local axis so that the initial
// configuration can violate the constraint slightly. The joint translation is zero
// when the local anchor points coincide in world space.
@[typedef] 
struct C.b2PrismaticJointDef {
pub mut:	
	// The first attached body
	bodyIdA BodyId 

	// The second attached body
	bodyIdB BodyId

	// The local anchor point relative to bodyA's origin
	localAnchorA Vec2

	// The local anchor point relative to bodyB's origin
	localAnchorB Vec2

	// The local translation unit axis in bodyA
	localAxisA Vec2

	// The constrained angle between the bodies: bodyB_angle - bodyA_angle
	referenceAngle f32

	// Enable a linear spring along the prismatic joint axis
	enableSpring bool

	// The spring stiffness Hertz, cycles per second
	hertz f32

	// The spring damping ratio, non-dimensional
	dampingRatio f32

	// Enable/disable the joint limit
	enableLimit bool

	// The lower translation limit
	lowerTranslation f32

	// The upper translation limit
	upperTranslation f32

	// Enable/disable the joint motor
	enableMotor bool

	// The maximum motor force, typically in newtons
	maxMotorForce f32

	// The desired motor speed, typically in meters per second
	motorSpeed f32

	// Set this flag to true if the attached bodies should collide
	collideConnected bool

	// User data pointer
	userData voidptr

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type PrismaticJointDef = C.b2PrismaticJointDef

// Revolute joint definition
//
// This requires defining an anchor point where the bodies are joined.
// The definition uses local anchor points so that the
// initial configuration can violate the constraint slightly. You also need to
// specify the initial relative angle for joint limits. This helps when saving
// and loading a game.
// The local anchor points are measured from the body's origin
// rather than the center of mass because:
// 1. you might not know where the center of mass will be
// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
// @ingroup revolute_joint
@[typedef] 
struct C.b2RevoluteJointDef {
pub mut:
	// The first attached body
	bodyIdA BodyId 

	// The second attached body
	bodyIdB BodyId

	// The local anchor point relative to bodyA's origin
	localAnchorA Vec2

	// The local anchor point relative to bodyB's origin
	localAnchorB Vec2

	// The bodyB angle minus bodyA angle in the reference state (radians).
	// This defines the zero angle for the joint limit.
	referenceAngle f32

	// Enable a rotational spring on the revolute hinge axis
	enableSpring bool

	// The spring stiffness Hertz, cycles per second
	hertz f32

	// The spring damping ratio, non-dimensional
	dampingRatio f32

	// A flag to enable joint limits
	enableLimit bool

	// The lower angle for the joint limit in radians
	lowerAngle f32

	// The upper angle for the joint limit in radians
	upperAngle f32

	// A flag to enable the joint motor
	enableMotor bool

	// The maximum motor torque, typically in newton-meters
	maxMotorTorque f32

	// The desired motor speed in radians per second
	motorSpeed f32

	// Scale the debug draw
	drawSize f32

	// Set this flag to true if the attached bodies should collide
	collideConnected bool

	// User data pointer
	userData voidptr

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type RevoluteJointDef = C.b2RevoluteJointDef

// Weld joint definition
//
// A weld joint connect to bodies together rigidly. This constraint provides springs to mimic
//	soft-body simulation.
// @note The approximate solver in Box2D cannot hold many bodies together rigidly
// @ingroup weld_joint
@[typedef] 
struct C.b2WeldJointDef {
pub mut:
	// The first attached body
	bodyIdA BodyId 

	// The second attached body
	bodyIdB BodyId 

	// The local anchor point relative to bodyA's origin
	localAnchorA Vec2 

	// The local anchor point relative to bodyB's origin
	localAnchorB Vec2

	// The bodyB angle minus bodyA angle in the reference state (radians)
	referenceAngle f32

	// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
	linearHertz f32

	// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	angularHertz f32

	// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	linearDampingRatio f32

	// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	angularDampingRatio f32

	// Set this flag to true if the attached bodies should collide
	collideConnected bool

	// User data pointer
	userData voidptr

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type WeldJointDef = C.b2WeldJointDef

// Wheel joint definition
//
// This requires defining a line of motion using an axis and an anchor point.
// The definition uses local  anchor points and a local axis so that the initial
// configuration can violate the constraint slightly. The joint translation is zero
// when the local anchor points coincide in world space.
// @ingroup wheel_joint
@[typedef] 
struct C.b2WheelJointDef {
pub mut:
	// The first attached body
	bodyIdA BodyId 

	// The second attached body
	bodyIdB BodyId

	// The local anchor point relative to bodyA's origin
	localAnchorA Vec2 

	// The local anchor point relative to bodyB's origin
	localAnchorB Vec2 

	// The local translation unit axis in bodyA
	localAxisA Vec2 

	// Enable a linear spring along the local axis
	enableSpring bool

	// Spring stiffness in Hertz
	hertz f32

	// Spring damping ratio, non-dimensional
	dampingRatio f32

	// Enable/disable the joint linear limit
	enableLimit bool

	// The lower translation limit
	lowerTranslation f32

	// The upper translation limit
	upperTranslation f32

	// Enable/disable the joint rotational motor
	enableMotor bool

	// The maximum motor torque, typically in newton-meters
	maxMotorTorquef f32

	// The desired motor speed in radians per second
	motorSpeed f32

	// Set this flag to true if the attached bodies should collide
	collideConnected bool

	// User data pointer
	userData voidptr

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int
}

pub type WheelJointDef = C.b2WheelJointDef

// A begin touch event is generated when a shape starts to overlap a sensor shape.
@[typedef] 
struct C.b2SensorBeginTouchEvent {
pub mut:
	// The id of the sensor shape
	sensorShapeId ShapeId

	// The id of the dynamic shape that began touching the sensor shape
	visitorShapeId ShapeId 
}

pub type SensorBeginTouchEvent = C.b2SensorBeginTouchEvent

// An end touch event is generated when a shape stops overlapping a sensor shape.
@[typedef] 
struct C.b2SensorEndTouchEvent {
pub mut:
	// The id of the sensor shape
	sensorShapeId ShapeId 

	// The id of the dynamic shape that stopped touching the sensor shape
	visitorShapeId ShapeId
}

pub type SensorEndTouchEvent = C.b2SensorEndTouchEvent

// Sensor events are buffered in the Box2D world and are available
//	as begin/end overlap event arrays after the time step is complete.
//	Note: these may become invalid if bodies and/or shapes are destroyed
@[typedef] 
struct C.b2SensorEvents {
pub mut:
	// Array of sensor begin touch events
	 beginEvents &SensorBeginTouchEvent

	// Array of sensor end touch events
	endEvents &SensorBeginTouchEvent

	// The number of begin touch events
	beginCount int

	// The number of end touch events
	endCount int
}

pub type SensorEvents = C.b2SensorEvents

// A begin touch event is generated when two shapes begin touching.
@[typedef] 
struct C.b2ContactBeginTouchEvent {
pub mut:
	// Id of the first shape
	shapeIdA ShapeId 

	// Id of the second shape
	shapeIdB ShapeId
}

pub type ContactBeginTouchEvent = C.b2ContactBeginTouchEvent 

// An end touch event is generated when two shapes stop touching.
@[typedef] 
struct C.b2ContactEndTouchEvent {
pub mut:
	// Id of the first shape
	shapeIdA ShapeId 

	// Id of the second shape
	shapeIdB ShapeId
}

pub type ContactEndTouchEvent = C.b2ContactEndTouchEvent

// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
@[typedef] 
struct C.b2ContactHitEvent {
pub mut:
	// Id of the first shape
	shapeIdA ShapeId

	// Id of the second shape
	shapeIdB ShapeId

	// Point where the shapes hit
	point Vec2

	// Normal vector pointing from shape A to shape B
	normal Vec2

	// The speed the shapes are approaching. Always positive. Typically in meters per second.
	approachSpeed f32
}

pub type ContactHitEvent = C.b2ContactHitEvent

// Contact events are buffered in the Box2D world and are available
//	as event arrays after the time step is complete.
//	Note: these may become invalid if bodies and/or shapes are destroyed
@[typedef] 
struct C.b2ContactEvents {
pub mut:
	// Array of begin touch events
	beginEvents &ContactBeginTouchEvent

	// Array of end touch events
	endEvents &ContactEndTouchEvent

	// Array of hit events
	hitEvents &ContactHitEvent

	// Number of begin touch events
	beginCount int

	// Number of end touch events
	endCount int

	// Number of hit events
	hitCount int
}

pub type ContactEvents = C.b2ContactEvents

// Body move events triggered when a body moves.
// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
// This also has a flag to indicate that the body went to sleep so the application can also
// sleep that actor/entity/object associated with the body.
// On the other hand if the flag does not indicate the body went to sleep then the application
// can treat the actor/entity/object associated with the body as awake.
//	This is an efficient way for an application to update game object transforms rather than
//	calling functions such as b2Body_GetTransform() because this data is delivered as a contiguous array
//	and it is only populated with bodies that have moved.
//	@note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
@[typedef] 
struct C.b2BodyMoveEvent {
pub mut:
	transform Transform
	bodyId BodyId 
	userData voidptr
	fellAsleep bool
}

pub type BodyMoveEvent = C.b2BodyMoveEvent

// Body events are buffered in the Box2D world and are available
//	as event arrays after the time step is complete.
//	Note: this date becomes invalid if bodies are destroyed
@[typedef] 
struct C.b2BodyEvents {
pub mut:
	// Array of move events
	moveEvents &BodyMoveEvent

	// Number of move events
	moveCount int
}

pub type BodyEvents = C.b2BodyEvents

// The contact data for two shapes. By convention the manifold normal points
//	from shape A to shape B.
//	@see b2Shape_GetContactData() and b2Body_GetContactData()
@[typedef] 
struct C.b2ContactData {
	shapeIdA ShapeId
	shapeIdB ShapeId
	manifold Manifold 
}

pub type ContactData = C.b2ContactData

pub type CustomFilterFcn = fn(shapeIdA ShapeId, shapeIdB ShapeId, context voidptr) bool
pub type PreSolveFcn = fn(shapeIdA ShapeId, shapeIdB ShapeId, manifold Manifold, context voidptr) bool
pub type OverlapResultFcn = fn(shapeId ShapeId, context voidptr) bool
pub type CastResultFcn = fn(shapeId ShapeId, point Vec2, normal Vec2, fraction f32, context voidptr) f32

// These colors are used for debug draw.
pub enum HexColor {
	color_alice_blue = 0xf0f8ff
	color_antique_white = 0xfaebd7
	color_aqua = 0x00ffff
	color_aquamarine = 0x7fffd4
	color_azure = 0xf0ffff
	color_beige = 0xf5f5dc
	color_bisque = 0xffe4c4
	color_black = 0x000000
	color_blanched_almond = 0xffebcd
	color_blue = 0x0000ff
	color_blue_violet = 0x8a2be2
	color_brown = 0xa52a2a
	color_burlywood = 0xdeb887
	color_cadet_blue = 0x5f9ea0
	color_chartreuse = 0x7fff00
	color_chocolate = 0xd2691e
	color_coral = 0xff7f50
	color_cornflower_blue = 0x6495ed
	color_cornsilk = 0xfff8dc
	color_crimson = 0xdc143c
	// color_cyan = 0x00ffff
	color_dark_blue = 0x00008b
	color_dark_cyan = 0x008b8b
	color_dark_goldenrod = 0xb8860b
	color_dark_gray = 0xa9a9a9
	color_dark_green = 0x006400
	color_dark_khaki = 0xbdb76b
	color_dark_magenta = 0x8b008b
	color_dark_olive_green = 0x556b2f
	color_dark_orange = 0xff8c00
	color_dark_orchid = 0x9932cc
	color_dark_red = 0x8b0000
	color_dark_salmon = 0xe9967a
	color_dark_sea_green = 0x8fbc8f
	color_dark_slate_blue = 0x483d8b
	color_dark_slate_gray = 0x2f4f4f
	color_dark_turquoise = 0x00ced1
	color_dark_violet = 0x9400d3
	color_deep_pink = 0xff1493
	color_deep_sky_blue = 0x00bfff
	color_dim_gray = 0x696969
	color_dodger_blue = 0x1e90ff
	color_firebrick = 0xb22222
	color_floral_white = 0xfffaf0
	color_forest_green = 0x228b22
	color_fuchsia = 0xff00ff
	color_gainsboro = 0xdcdcdc
	color_ghost_white = 0xf8f8ff
	color_gold = 0xffd700
	color_goldenrod = 0xdaa520
	color_gray = 0xbebebe
	color_gray1 = 0x1a1a1a
	color_gray2 = 0x333333
	color_gray3 = 0x4d4d4d
	color_gray4 = 0x666666
	color_gray5 = 0x7f7f7f
	color_gray6 = 0x999999
	color_gray7 = 0xb3b3b3
	color_gray8 = 0xcccccc
	color_gray9 = 0xe5e5e5
	color_green = 0x00ff00
	color_green_yellow = 0xadff2f
	color_honeydew = 0xf0fff0
	color_hot_pink = 0xff69b4
	color_indian_red = 0xcd5c5c
	color_indigo = 0x4b0082
	color_ivory = 0xfffff0
	color_khaki = 0xf0e68c
	color_lavender = 0xe6e6fa
	color_lavender_blush = 0xfff0f5
	color_lawn_green = 0x7cfc00
	color_lemon_chiffon = 0xfffacd
	color_light_blue = 0xadd8e6
	color_light_coral = 0xf08080
	color_light_cyan = 0xe0ffff
	color_light_goldenrod = 0xeedd82
	color_light_goldenrod_yellow = 0xfafad2
	color_light_gray = 0xd3d3d3
	color_light_green = 0x90ee90
	color_light_pink = 0xffb6c1
	color_light_salmon = 0xffa07a
	color_light_sea_green = 0x20b2aa
	color_light_sky_blue = 0x87cefa
	color_light_slate_blue = 0x8470ff
	color_light_slate_gray = 0x778899
	color_light_steel_blue = 0xb0c4de
	color_light_yellow = 0xffffe0
	// color_lime = 0x00ff00
	color_lime_green = 0x32cd32
	color_linen = 0xfaf0e6
	// color_magenta = 0xff00ff
	color_maroon = 0xb03060
	color_medium_aquamarine = 0x66cdaa
	color_medium_blue = 0x0000cd
	color_medium_orchid = 0xba55d3
	color_medium_purple = 0x9370db
	color_medium_sea_green = 0x3cb371
	color_medium_slate_blue = 0x7b68ee
	color_medium_spring_green = 0x00fa9a
	color_medium_turquoise = 0x48d1cc
	color_medium_violet_red = 0xc71585
	color_midnight_blue = 0x191970
	color_mint_cream = 0xf5fffa
	color_misty_rose = 0xffe4e1
	color_moccasin = 0xffe4b5
	color_navajo_white = 0xffdead
	color_navy = 0x000080
	// color_navy_blue = 0x000080
	color_old_lace = 0xfdf5e6
	color_olive = 0x808000
	color_olive_drab = 0x6b8e23
	color_orange = 0xffa500
	color_orange_red = 0xff4500
	color_orchid = 0xda70d6
	color_pale_goldenrod = 0xeee8aa
	color_pale_green = 0x98fb98
	color_pale_turquoise = 0xafeeee
	color_pale_violet_red = 0xdb7093
	color_papaya_whip = 0xffefd5
	color_peach_puff = 0xffdab9
	color_peru = 0xcd853f
	color_pink = 0xffc0cb
	color_plum = 0xdda0dd
	color_powder_blue = 0xb0e0e6
	color_purple = 0xa020f0
	color_rebecca_purple = 0x663399
	color_red = 0xff0000
	color_rosy_brown = 0xbc8f8f
	color_royal_blue = 0x4169e1
	color_saddle_brown = 0x8b4513
	color_salmon = 0xfa8072
	color_sandy_brown = 0xf4a460
	color_sea_green = 0x2e8b57
	color_seashell = 0xfff5ee
	color_sienna = 0xa0522d
	color_silver = 0xc0c0c0
	color_sky_blue = 0x87ceeb
	color_slate_blue = 0x6a5acd
	color_slate_gray = 0x708090
	color_snow = 0xfffafa
	color_spring_green = 0x00ff7f
	color_steel_blue = 0x4682b4
	color_tan = 0xd2b48c
	color_teal = 0x008080
	color_thistle = 0xd8bfd8
	color_tomato = 0xff6347
	color_turquoise = 0x40e0d0
	color_violet = 0xee82ee
	color_violet_red = 0xd02090
	color_wheat = 0xf5deb3
	color_white = 0xffffff
	color_white_smoke = 0xf5f5f5
	color_yellow = 0xffff00
	color_yellow_green = 0x9acd32
}

// This struct holds callbacks you can implement to draw a Box2D world.
//	@ingroup world
@[typedef] 
struct C.b2DebugDraw {
pub mut:
	// Draw a closed polygon provided in CCW order.
	DrawPolygon fn(vertices Vec2, vertexCount int, color HexColor, context voidptr)

	// Draw a solid closed polygon provided in CCW order.
	DrawSolidPolygon fn(transform Transform, vertices &Vec2, vertexCount int, radius f32, color HexColor ,context voidptr)

	// Draw a circle.
	DrawCircle fn(center Vec2, radius f32, color  HexColor, context voidptr)

	// Draw a solid circle.
	DrawSolidCircle fn(transform Transform , radius f32, color HexColor, context voidptr)

	// Draw a capsule.
	DrawCapsule fn(p1 Vec2, p2 Vec2, radius f32, color HexColor, context voidptr)

	// Draw a solid capsule.
	DrawSolidCapsule fn(p1 Vec2 , p2 Vec2 , radius f32, color HexColor, context voidptr)

	// Draw a line segment.
	DrawSegment fn(p1 Vec2, p2 Vec2 , color HexColor , context voidptr)

	// Draw a transform. Choose your own length scale.
	DrawTransform fn(transform Transform , context voidptr)

	// Draw a point.
	DrawPoint fn(p Vec2, size f32, color HexColor, context voidptr)

	// Draw a string.
	DrawString fn(p Vec2, s &char, context voidptr)

	// Bounds to use if restricting drawing to a rectangular region
	drawingBounds AABB 

	// Option to restrict drawing to a rectangular region. May suffer from unstable depth sorting.
	useDrawingBounds bool

	// Option to draw shapes
	drawShapes bool

	// Option to draw joints
	drawJoints bool

	// Option to draw additional information for joints
	drawJointExtras bool

	// Option to draw the bounding boxes for shapes
	drawAABBs bool

	// Option to draw the mass and center of mass of dynamic bodies
	drawMass bool

	// Option to draw contact points
	drawContacts bool

	// Option to visualize the graph coloring used for contacts and joints
	drawGraphColors bool

	// Option to draw contact normals
	drawContactNormals bool

	// Option to draw contact normal impulses
	drawContactImpulses bool

	// Option to draw contact friction impulses
	drawFrictionImpulses bool

	// User context that is passed as an argument to drawing callback functions
	context voidptr
}

pub type DebugDraw = C.b2DebugDraw

// Use this to initialize your world definition
// @ingroup world
fn C.b2DefaultWorldDef() WorldDef
@[inline]
pub fn default_world_def() WorldDef {
	return C.b2DefaultWorldDef()
}

// Use this to initialize your body definition
// @inbody
fn C.b2DefaultBodyDef() BodyDef
@[inline]
pub fn default_body_def() BodyDef {
	return C.b2DefaultBodyDef()
}

// Use this to initialize your filter
// @ingroup shape
fn C.b2DefaultFilter() Filter
@[inline]
pub fn default_filter() Filter {
	return C.b2DefaultFilter()
}

// Use this to initialize your query filter
// @ingroup shape
fn C.b2DefaultQueryFilter() QueryFilter
@[inline]
pub fn default_query_filter() QueryFilter {
	return C.b2DefaultQueryFilter()
}

// Use this to initialize your shape definition
// @ingroup shape
fn C.b2DefaultShapeDef() ShapeDef
@[inline]
pub fn default_shape_def() ShapeDef {
	return C.b2DefaultShapeDef()
}

// Use this to initialize your chain definition
// @ingroup shape
fn C.b2DefaultChainDef() ChainDef
@[inline]
pub fn default_chain_def() ChainDef {
	return C.b2DefaultChainDef()
}

// Use this to initialize your joint definition
// @ingroup distance_joint
fn C.b2DefaultDistanceJointDef() DistanceJointDef
@[inline]
pub fn default_distance_joint_def() DistanceJointDef {
	return C.b2DefaultDistanceJointDef()
}

// Use this to initialize your joint definition
// @ingroup motor_joint
fn C.b2DefaultMotorJointDef() MotorJointDef
@[inline]
pub fn default_motor_joint_def() MotorJointDef {
	return C.b2DefaultMotorJointDef()
}

// Use this to initialize your joint definition
// @ingroup mouse_joint
fn C.b2DefaultMouseJointDef() MouseJointDef
@[inline]
pub fn default_mouse_joint_def() MouseJointDef {
	return C.b2DefaultMouseJointDef() 
}

// Use this to initialize your joint definition
// @ingroupd prismatic_joint
fn C.b2DefaultPrismaticJointDef() PrismaticJointDef
@[inline]
pub fn default_prismatic_joint_def() PrismaticJointDef {
	return C.b2DefaultPrismaticJointDef()
}

// Use this to initialize your joint definition.
// @ingroup revolute_joint
fn C.b2DefaultRevoluteJointDef() RevoluteJointDef 
@[inline]
pub fn default_revolute_joint_def() RevoluteJointDef {
	return C.b2DefaultRevoluteJointDef()
}

// Use this to initialize your joint definition
// ingroup weld_joint
fn C.b2DefaultWeldJointDef() WeldJointDef
@[inline]
pub fn default_weld_joint_def() WeldJointDef {
	return C.b2DefaultWeldJointDef()
}

// Use this to initialize your joint definition
// ingroup wheel_joint
fn C.b2DefaultWheelJointDef() WheelJointDef 
@[inline]
pub fn default_wheel_joint_def() WheelJointDef {
	return C.b2DefaultWheelJointDef()
}

// end types

// start box2d

// Create a world for rigid body simulation. A world contains bodies, shapes, and constraints. You make create
// up to 128 worlds. Each world is completely independent and may be simulated in parallel.
// - return the world id.
fn C.b2CreateWorld(def &WorldDef) WorldId 
@[inline]
pub fn create_world(def &WorldDef) WorldId  {
	return C.b2CreateWorld(def) 
}

// Destroy a world
fn C.b2DestroyWorld(worldId WorldId)
@[inline]
pub fn destroy_world(worldId WorldId) {
	C.b2DestroyWorld(worldId)
}

// World id validation. Provides validation for up to 64K allocations.

fn C.b2World_IsValid(id WorldId) bool
@[inline]
pub fn world_is_valid(id WorldId) bool {
	return C.b2World_IsValid(id)
}
// Simulate a world for one time step. This performs collision detection, integration, and constraint solution.
// @param worldId The world to simulate
// @param timeStep The amount of time to simulate, this should be a fixed number. Typically 1/60.
// @param subStepCount The number of sub-steps, increasing the sub-step count can increase accuracy. Typically 4.
fn C.b2World_Step(worldId WorldId, timeStep f32, subStepCount int)
@[inline]
pub fn world_step(worldId WorldId, timeStep f32, subStepCount int) {
	C.b2World_Step(worldId, timeStep, subStepCount)
}
// Call this to draw shapes and other debug draw data
fn C.b2World_Draw(worldId WorldId, draw &DebugDraw)
@[inline]
pub fn world_draw(worldId WorldId, draw &DebugDraw) {
	C.b2World_Draw(worldId, draw)
}
// Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
fn C.b2World_GetBodyEvents(worldId WorldId) BodyEvents
@[inline]
pub fn world_get_body_events(worldId WorldId) BodyEvents {
	return C.b2World_GetBodyEvents(worldId)
}
// Get 
// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
fn C.b2World_GetSensorEvents(worldId WorldId) SensorEvents
@[inline]
pub fn world_get_sensor_events(worldId WorldId) SensorEvents {
	return C.b2World_GetSensorEvents(worldId)
}
// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
fn C.b2World_GetContactEvents(worldId WorldId) ContactEvents
@[inline]
pub fn world_get_contact_events(worldId WorldId) ContactEvents {
	return C.b2World_GetContactEvents(worldId)
}
// Overlap test for all shapes that *potentially* overlap the provided AABB
fn C.b2World_OverlapAABB(worldId WorldId, aabb AABB, filter QueryFilter, fcn &OverlapResultFcn, context voidptr)
@[inline]
pub fn world_overlap_aabb(worldId WorldId, aabb AABB, filter QueryFilter, fcn &OverlapResultFcn, context voidptr) {
	C.b2World_OverlapAABB(worldId, aabb, filter, fcn, context)
}
// Overlap test for for all shapes that overlap the provided circle
fn C.b2World_OverlapCircle(worldId WorldId, circle &Circle, transform Transform, filter QueryFilter, fcn &OverlapResultFcn, context voidptr)
@[inline]
pub fn world_overlap_circle(worldId WorldId, circle &Circle, transform Transform, filter QueryFilter, fcn &OverlapResultFcn, context voidptr) {
	C.b2World_OverlapCircle(worldId, circle, transform, filter, fcn, context)
}
// Overlap test for all shapes that overlap the provided capsule
fn C.b2World_OverlapCapsule(worldId WorldId, capsule &Capsule, transform Transform, filter QueryFilter, fcn &OverlapResultFcn, context voidptr)
@[inline]
pub fn world_overlap_capsule(worldId WorldId, capsule &Capsule, transform Transform, filter QueryFilter, fcn &OverlapResultFcn, context voidptr) {
	C.b2World_OverlapCapsule(worldId, capsule, transform, filter, fcn, context)
}
// Overlap test for all shapes that overlap the provided polygon
fn C.b2World_OverlapPolygon(worldId WorldId, polygon &Polygon, transform Transform, filter QueryFilter, fcn &OverlapResultFcn, context voidptr)
@[inline]
pub fn world_overlap_polygon(worldId WorldId, polygon &Polygon, transform Transform, filter QueryFilter, fcn &OverlapResultFcn, context voidptr) {
	C.b2World_OverlapPolygon(worldId, polygon, transform, filter, fcn, context)
}
// Cast a ray into the world to collect shapes in the path of the ray.
// Your callback function controls whether you get the closest point, any point, or n-points.
// The ray-cast ignores shapes that contain the starting point.
//	@param worldId The world to cast the ray against
//	@param origin The start point of the ray
//	@param translation The translation of the ray from the start point to the end point
//	@param filter Contains bit flags to filter unwanted shapes from the results
// @param fcn A user implemented callback function
// @param context A user context that is passed along to the callback function
//	@note The callback function may receive shapes in any order
fn C.b2World_CastRay(worldId WorldId, origin Vec2, translation Vec2, filter QueryFilter, fcn &CastResultFcn, context voidptr)
@[inline]
pub fn world_cast_ray(worldId WorldId, origin Vec2, translation Vec2, filter QueryFilter, fcn &CastResultFcn, context voidptr) {
	C.b2World_CastRay(worldId, origin, translation, filter, fcn, context)
}
// Cast a ray into the world to collect the closest hit. This is a convenience function.
// This is less general than b2World_CastRay() and does not allow for custom filtering.
fn C.b2World_CastRayClosest(worldId WorldId, origin Vec2, translation Vec2, filter QueryFilter) RayResult
@[inline]
pub fn world_cast_ray_closest(worldId WorldId, origin Vec2, translation Vec2, filter QueryFilter) RayResult {
	return C.b2World_CastRayClosest(worldId, origin, translation, filter)
}
// Cast a circle through the world. Similar to a cast ray except that a circle is cast instead of a point.
fn C.b2World_CastCircle(worldId WorldId, circle &Circle, originTransform Transform, translation Vec2, filter QueryFilter, fcn &CastResultFcn, context voidptr)
@[inline]
pub fn world_cast_circle(worldId WorldId, circle &Circle, originTransform Transform, translation Vec2, filter QueryFilter, fcn &CastResultFcn, context voidptr) {
	C.b2World_CastCircle(worldId, circle, originTransform, translation, filter, fcn, context)
}
// Cast a capsule through the world. Similar to a cast ray except that a capsule is cast instead of a point.
fn C.b2World_CastCapsule(worldId WorldId, capsule &Capsule, originTransform Transform, translation Vec2, filter QueryFilter, fcn &CastResultFcn, context voidptr)
@[inline]
pub fn world_cast_capsule(worldId WorldId, capsule &Capsule, originTransform Transform, translation Vec2, filter QueryFilter, fcn &CastResultFcn, context voidptr) {
	C.b2World_CastCapsule(worldId, capsule, originTransform, translation, filter, fcn, context)
}
// Cast a polygon through the world. Similar to a cast ray except that a polygon is cast instead of a point.
fn C.b2World_CastPolygon(worldId WorldId, polygon &Polygon, originTransform Transform, translation Vec2, filter QueryFilter, fcn &CastResultFcn, context voidptr)
@[inline]
pub fn world_cast_polygon(worldId WorldId, polygon &Polygon, originTransform Transform, translation Vec2, filter QueryFilter, fcn &CastResultFcn, context voidptr) {
	C.b2World_CastPolygon(worldId, polygon, originTransform, translation, filter, fcn, context)
}
// Enable/disable sleep. If your application does not need sleeping, you can gain some performance
//	by disabling sleep completely at the world level.
//	@see b2WorldDef
fn C.b2World_EnableSleeping(worldId WorldId, flag bool)
@[inline]
pub fn world_enable_sleeping(worldId WorldId, flag bool) {
	C.b2World_EnableSleeping(worldId, flag)
}
// Enable/disable continuous collision between dynamic and static bodies. Generally you should keep continuous
// collision enabled to prevent fast moving objects from going through static objects. The performance gain from
//	disabling continuous collision is minor.
//	@see b2WorldDef
fn C.b2World_EnableContinuous(worldId WorldId, flag bool)
@[inline]
pub fn world_enable_continuous(worldId WorldId, flag bool) {
	C.b2World_EnableContinuous(worldId, flag)
}
// Adjust the restitution threshold. It is recommended not to make this value very small
//	because it will prevent bodies from sleeping. Typically in meters per second.
//	@see b2WorldDef
fn C.b2World_SetRestitutionThreshold(worldId WorldId, value f32)
@[inline]
pub fn world_set_restitution_threshold(worldId WorldId, value f32) {
	C.b2World_SetRestitutionThreshold(worldId, value)
}
// Adjust the hit event threshold. This controls the collision velocity needed to generate a b2ContactHitEvent.
// Typically in meters per second.
//	@see b2WorldDef::hitEventThreshold
fn C.b2World_SetHitEventThreshold(worldId WorldId, value f32)
@[inline]
pub fn world_set_hit_event_threshold(worldId WorldId, value f32) {
	C.b2World_SetHitEventThreshold(worldId, value)
}
// Register the custom filter callback. This is optional.
fn C.b2World_SetCustomFilterCallback(worldId WorldId, fcn &CustomFilterFcn, context voidptr)
@[inline]
pub fn world_set_custom_filter_callback(worldId WorldId, fcn &CustomFilterFcn, context voidptr) {
	C.b2World_SetCustomFilterCallback(worldId, fcn, context)
}
// Register the pre-solve callback. This is optional.b2World_SetPreSolveCallback(worldId WorldId, fcn &PreSolveFcn, context voidptr)
fn C.b2World_SetPreSolveCallback(worldId WorldId, fcn &PreSolveFcn, context voidptr)
@[inline]
pub fn world_set_pre_solve_callback(worldId WorldId, fcn &PreSolveFcn, context voidptr) {
	C.b2World_SetPreSolveCallback(worldId, fcn, context)
}
// Set the gravity vector for the entire world. Box2D has no concept of an up direction and this
// is left as a decision for the application. Typically in m/s^2.
//	@see b2WorldDef
fn C.b2World_SetGravity(worldId WorldId, gravity Vec2)
@[inline]
pub fn world_set_gravity(worldId WorldId, gravity Vec2) {
	C.b2World_SetGravity(worldId, gravity)
}
// Get the gravity vector
fn C.b2World_GetGravity(worldId WorldId) Vec2
@[inline]
pub fn world_get_gravity(worldId WorldId) Vec2 {
	return C.b2World_GetGravity(worldId)
}
// Apply a radial explosion
//	@param worldId The world id
//	@param position The center of the explosion
//	@param radius The radius of the explosion
//	@param impulse The impulse of the explosion, typically in kg * m / s or N * s.
fn C.b2World_Explode(worldId WorldId, position Vec2, radius f32, impulse f32)
@[inline]
pub fn world_explode(worldId WorldId, position Vec2, radius f32, impulse f32) {
	C.b2World_Explode(worldId, position, radius, impulse)
}

// Adjust contact tuning parameters
//	@param worldId The world id
// @param hertz The contact stiffness (cycles per second)
// @param dampingRatio The contact bounciness with 1 being critical damping (non-dimensional)
// @param pushVelocity The maximum contact constraint push out velocity (meters per second)
//	@note Advanced feature

fn C.b2World_SetContactTuning(worldId WorldId, hertz f32, dampingRatio f32, pushVelocity f32)
@[inline]
pub fn world_set_contact_tuning(worldId WorldId, hertz f32, dampingRatio f32, pushVelocity f32){
	C.b2World_SetContactTuning(worldId, hertz, dampingRatio, pushVelocity)
}

// Enable/disable constraint warm starting. Advanced feature for testing. Disabling
//	sleeping greatly reduces stability and provides no performance gain.
fn C.b2World_EnableWarmStarting(worldId WorldId, flag bool)
@[inline]
pub fn world_enable_warm_starting(worldId WorldId, flag bool) {
	C.b2World_EnableWarmStarting(worldId, flag)
}

// Get the current world performance profile 
fn C.b2World_GetProfile(worldId WorldId) Profile
@[inline]
pub fn world_get_profile(worldId WorldId) Profile {
	return C.b2World_GetProfile(worldId)
}

// Get world counters and sizes
fn C.b2World_GetCounters(worldId WorldId) Counters
@[inline]
pub fn world_get_counters(worldId WorldId) Counters {
	return C.b2World_GetCounters(worldId)
}

// Dump memory stats to box2d_memory.txt
fn C.b2World_DumpMemoryStats(worldId WorldId)
@[inline]
pub fn world_dump_memory_stats(worldId WorldId) {
	C.b2World_DumpMemoryStats(worldId)
}

/** @} */

/**
 * @defgroup body Body
 * This is the body API.
 * @{
 */

// Create a rigid body given a definition. No reference to the definition is retained. So you can create the definition
//	on the stack and pass it as a pointer.
//	@code{.c}
//	b2BodyDef bodyDef = b2DefaultBodyDef();
//	b2BodyId myBodyId = b2CreateBody(myWorldId, &bodyDef);
//	@endcode
// @warning This function is locked during callbacks.
fn C.b2CreateBody(worldId WorldId, def &BodyDef) BodyId
@[inline]
pub fn create_body(worldId WorldId, def &BodyDef) BodyId {
	return C.b2CreateBody(worldId, def) 
}

// Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
//	Do not keep references to the associated shapes and joints.
fn C.b2DestroyBody(bodyId BodyId)
@[inline]
pub fn destroy_body(bodyId BodyId) {
	C.b2DestroyBody(bodyId)
}

// Body identifier validation. Can be used to detect orphaned ids. Provides validation for up to 64K allocations.
fn C.b2Body_IsValid(id BodyId) bool
@[inline]
pub fn body_is_valid(id BodyId) bool {
	return C.b2Body_IsValid(id)
}

// Get the body type: static, kinematic, or dynamic
fn C.b2Body_GetType(bodyId BodyId) BodyType
@[inline]
pub fn body_get_type(bodyId BodyId) BodyType {
	return C.b2Body_GetType(bodyId)
}

// Change the body type. This is an expensive operation. This automatically updates the mass
//	properties regardless of the automatic mass setting.
fn C.b2Body_SetType(bodyId BodyId, types BodyType)
@[inline]
pub fn body_set_type(bodyId BodyId, types BodyType){
	C.b2Body_SetType(bodyId, types)
}

// Set the user data for a body
fn C.b2Body_SetUserData(bodyId BodyId, userData voidptr)
@[inline]
pub fn body_set_user_data(bodyId BodyId, userData voidptr) {
	C.b2Body_SetUserData(bodyId, userData)
}

// Get the user data stored in a body
fn C.b2Body_GetUserData(bodyId BodyId) voidptr
@[inline]
pub fn body_get_user_data(bodyId BodyId) voidptr {
	return C.b2Body_GetUserData(bodyId)
}

// Get the world position of a body. This is the location of the body origin.
fn C.b2Body_GetPosition(bodyId BodyId) Vec2
@[inline]
pub fn body_get_position(bodyId BodyId) Vec2 {
	return C.b2Body_GetPosition(bodyId) 
}

// Get the world rotation of a body as a cosine/sine pair (complex number)
fn C.b2Body_GetRotation(bodyId BodyId) Rot
@[inline]
pub fn body_get_rotation(bodyId BodyId) Rot {
	return C.b2Body_GetRotation(bodyId)
}

// Get the world transform of a body.
fn C.b2Body_GetTransform(bodyId BodyId) Transform
@[inline]
pub fn body_get_transform(bodyId BodyId) Transform {
	return C.b2Body_GetTransform(bodyId)
}

// Set the world transform of a body. This acts as a teleport and is fairly expensive.
// @note Generally you should create a body with then intended transform.
//	@see BodyDef::position and BodyDef::angle
fn C.b2Body_SetTransform(bodyId BodyId, position Vec2, rotation Rot)
@[inline]
pub fn body_set_transform(bodyId BodyId, position Vec2, rotation Rot) {
	C.b2Body_SetTransform(bodyId, position, rotation)
}

// Get a local point on a body given a world point
fn C.b2Body_GetLocalPoint(bodyId BodyId, worldPoint Vec2) Vec2
@[inline]
pub fn body_get_local_point(bodyId BodyId, worldPoint Vec2) Vec2 {
	return C.b2Body_GetLocalPoint(bodyId, worldPoint)
}

// Get a world point on a body given a local point
fn C.b2Body_GetWorldPoint(bodyId BodyId, localPoint Vec2) Vec2
@[inline]
pub fn body_get_world_point(bodyId BodyId, localPoint Vec2) Vec2 {
	return C.b2Body_GetWorldPoint(bodyId, localPoint)
}

// Get a local vector on a body given a world vector
fn C.b2Body_GetLocalVector(bodyId BodyId, worldVector Vec2) Vec2
@[inline]
pub fn body_get_local_vector(bodyId BodyId, worldVector Vec2) Vec2 {
	return C.b2Body_GetLocalVector(bodyId, worldVector)
}

// Get a world vector on a body given a local vector
fn C.b2Body_GetWorldVector(bodyId BodyId, localVector Vec2) Vec2
@[inline]
pub fn body_get_world_vector(bodyId BodyId, localVector Vec2) Vec2 {
	return C.b2Body_GetWorldVector(bodyId, localVector)
}

// Get the linear velocity of a body's center of mass. Typically in meters per second.
fn C.b2Body_GetLinearVelocity(bodyId BodyId) Vec2
@[inline]
pub fn body_get_linear_velocity(bodyId BodyId) Vec2 {
	return C.b2Body_GetLinearVelocity(bodyId)
}

// Get the angular velocity of a body in radians per second
fn C.b2Body_GetAngularVelocity(bodyId BodyId) f32
@[inline]
pub fn body_get_angular_velocity(bodyId BodyId) f32 {
	return C.b2Body_GetAngularVelocity(bodyId)
}

// Set the linear velocity of a body. Typically in meters per second.
fn C.b2Body_SetLinearVelocity(bodyId BodyId, linearVelocity Vec2)
@[inline]
pub fn body_set_linear_velocity(bodyId BodyId, linearVelocity Vec2) {
	C.b2Body_SetLinearVelocity(bodyId, linearVelocity)
}

// Set the angular velocity of a body in radians per second
fn C.b2Body_SetAngularVelocity(bodyId BodyId, angularVelocity f32)
@[inline]
pub fn body_set_angular_velocity(bodyId BodyId, angularVelocity f32) {
	C.b2Body_SetAngularVelocity(bodyId, angularVelocity)
}

// Apply a force at a world point. If the force is not applied at the center of mass,
// it will generate a torque and affect the angular velocity. This optionally wakes up the body.
//	The force is ignored if the body is not awake.
//	@param bodyId The body id
// @param force The world force vector, typically in newtons (N)
// @param point The world position of the point of application
// @param wake Option to wake up the body
fn C.b2Body_ApplyForce(bodyId BodyId, force Vec2, point Vec2, wake bool)
@[inline]
pub fn body_apply_force(bodyId BodyId, force Vec2, point Vec2, wake bool) {
	C.b2Body_ApplyForce(bodyId, force, point, wake)
}

// Apply a force to the center of mass. This optionally wakes up the body.
//	The force is ignored if the body is not awake.
//	@param bodyId The body id
// @param force the world force vector, usually in newtons (N).
// @param wake also wake up the body
fn C.b2Body_ApplyForceToCenter(bodyId BodyId, force Vec2, wake bool)
@[inline]
pub fn body_apply_force_to_center(bodyId BodyId, force Vec2, wake bool) {
	C.b2Body_ApplyForceToCenter(bodyId, force, wake)
}

// Apply a torque. This affects the angular velocity without affecting the linear velocity.
//	This optionally wakes the body. The torque is ignored if the body is not awake.
//	@param bodyId The body id
// @param torque about the z-axis (out of the screen), typically in N*m.
// @param wake also wake up the body
fn C.b2Body_ApplyTorque(bodyId BodyId, torque f32, wake bool)
@[inline]
pub fn body_apply_torque(bodyId BodyId, torque f32, wake bool) {
	C.b2Body_ApplyTorque(bodyId, torque, wake)
}

// Apply an impulse at a point. This immediately modifies the velocity.
// It also modifies the angular velocity if the point of application
// is not at the center of mass. This optionally wakes the body.
// The impulse is ignored if the body is not awake.
//	@param bodyId The body id
// @param impulse the world impulse vector, typically in N*s or kg*m/s.
// @param point the world position of the point of application.
// @param wake also wake up the body
//	@warning This should be used for one-shot impulses. If you need a steady force,
// use a force instead, which will work better with the sub-stepping solver.
fn C.b2Body_ApplyLinearImpulse(bodyId BodyId, impulse Vec2, point Vec2, wake bool)
@[inline]
pub fn body_apply_linear_impulse(bodyId BodyId, impulse Vec2, point Vec2, wake bool) {
	C.b2Body_ApplyLinearImpulse(bodyId, impulse, point, wake)
}

// Apply an impulse to the center of mass. This immediately modifies the velocity.
// The impulse is ignored if the body is not awake. This optionally wakes the body.
//	@param bodyId The body id
// @param impulse the world impulse vector, typically in N*s or kg*m/s.
// @param wake also wake up the body
//	@warning This should be used for one-shot impulses. If you need a steady force,
// use a force instead, which will work better with the sub-stepping solver.
fn C.b2Body_ApplyLinearImpulseToCenter(bodyId BodyId, impulse Vec2, wake bool)
@[inline]
pub fn body_apply_linear_impulse_to_center(bodyId BodyId, impulse Vec2, wake bool) {
	C.b2Body_ApplyLinearImpulseToCenter(bodyId, impulse, wake)
}

// Apply an angular impulse. The impulse is ignored if the body is not awake.
// This optionally wakes the body.
//	@param bodyId The body id
// @param impulse the angular impulse, typically in units of kg*m*m/s
// @param wake also wake up the body
//	@warning This should be used for one-shot impulses. If you need a steady force,
// use a force instead, which will work better with the sub-stepping solver.
fn C.b2Body_ApplyAngularImpulse(bodyId BodyId, impulse f32, wake bool)
@[inline]
pub fn body_apply_angular_impulse(bodyId BodyId, impulse f32, wake bool) {
	C.b2Body_ApplyAngularImpulse(bodyId, impulse, wake)
}

// Get the mass of the body, typically in kilograms
fn C.b2Body_GetMass(bodyId BodyId) f32
@[inline]
pub fn body_get_mass(bodyId BodyId) f32 {
	return C.b2Body_GetMass(bodyId)
}

// Get the inertia tensor of the body, typically in kg*m^2
fn C.b2Body_GetInertiaTensor(bodyId BodyId) f32
@[inline]
pub fn body_get_inertia_tensor(bodyId BodyId) f32 {
	return C.b2Body_GetInertiaTensor(bodyId)
}

// Get the center of mass position of the body in local space
fn C.b2Body_GetLocalCenterOfMass(bodyId BodyId) Vec2
@[inline]
pub fn body_get_local_center_of_mass(bodyId BodyId) Vec2 {
	return C.b2Body_GetLocalCenterOfMass(bodyId)
}

// Get the center of mass position of the body in world space
fn C.b2Body_GetWorldCenterOfMass(bodyId BodyId) Vec2
@[inline]
pub fn body_get_world_center_of_mass(bodyId BodyId) Vec2 {
	return C.b2Body_GetWorldCenterOfMass(bodyId)
}

// Override the body's mass properties. Normally this is computed automatically using the
//	shape geometry and density. This information is lost if a shape is added or removed or if the
//	body type changes.
fn C.b2Body_SetMassData(bodyId BodyId, massData MassData)
@[inline]
pub fn body_set_mass_data(bodyId BodyId, massData MassData) {
	C.b2Body_SetMassData(bodyId, massData)
}

// Get the mass data for a body
fn C.b2Body_GetMassData(bodyId BodyId) MassData
@[inline]
pub fn body_get_mass_data(bodyId BodyId) MassData {
	return C.b2Body_GetMassData(bodyId)
}

// This update the mass properties to the sum of the mass properties of the shapes.
// This normally does not need to be called unless you called SetMassData to override
// the mass and you later want to reset the mass.
//	You may also use this when automatic mass computation has been disabled.
//	You should call this regardless of body type.
fn C.b2Body_ApplyMassFromShapes(bodyId BodyId)
@[inline]
pub fn body_apply_mass_from_shapes(bodyId BodyId) {
	C.b2Body_ApplyMassFromShapes(bodyId)
}

// Set the automatic mass setting. Normally this is set in BodyDef before creation.
//	@see BodyDef::automaticMass
fn C.b2Body_SetAutomaticMass(bodyId BodyId, automaticMass bool)
@[inline]
pub fn body_set_automatic_mass(bodyId BodyId, automaticMass bool) {
	C.b2Body_SetAutomaticMass(bodyId, automaticMass)
}

// Get the automatic mass setting 
fn C.b2Body_GetAutomaticMass(bodyId BodyId) bool
@[inline]
pub fn body_get_automatic_mass(bodyId BodyId) bool {
	return C.b2Body_GetAutomaticMass(bodyId)
}

// Adjust the linear damping. Normally this is set in BodyDef before creation.
fn C.b2Body_SetLinearDamping(bodyId BodyId, linearDamping f32)
@[inline]
pub fn body_set_linear_damping(bodyId BodyId, linearDamping f32) {
	C.b2Body_SetLinearDamping(bodyId, linearDamping)
}

// Get the current linear damping.
fn C.b2Body_GetLinearDamping(bodyId BodyId) f32
@[inline]
pub fn body_get_linear_damping(bodyId BodyId) f32 {
	return C.b2Body_GetLinearDamping(bodyId)
}

// Adjust the angular damping. Normally this is set in BodyDef before creation.
fn C.b2Body_SetAngularDamping(bodyId BodyId, angularDamping f32)
@[inline]
pub fn body_set_angular_damping(bodyId BodyId, angularDamping f32) {
	C.b2Body_SetAngularDamping(bodyId, angularDamping)
}

// Get the current angular damping.
fn C.b2Body_GetAngularDamping(bodyId BodyId) f32
@[inline]
pub fn body_get_angular_damping(bodyId BodyId) f32  {
	return C.b2Body_GetAngularDamping(bodyId)
}

// Adjust the gravity scale. Normally this is set in BodyDef before creation.
//	@see BodyDef::gravityScale
fn C.b2Body_SetGravityScale(bodyId BodyId, gravityScale f32)
@[inline]
pub fn body_set_gravity_scale(bodyId BodyId, gravityScale f32) {
	C.b2Body_SetGravityScale(bodyId, gravityScale)
}

// Get the current gravity scale
fn C.b2Body_GetGravityScale(bodyId BodyId) f32
@[inline]
pub fn body_get_gravity_scale(bodyId BodyId) f32 {
	return C.b2Body_GetGravityScale(bodyId)
}
// @return true if this body is awake
fn C.b2Body_IsAwake(bodyId BodyId) bool
@[inline]
pub fn body_is_awake(bodyId BodyId) bool {
	return C.b2Body_IsAwake(bodyId)
}

// Wake a body from sleep. This wakes the entire island the body is touching.
//	@warning Putting a body to sleep will put the entire island of bodies touching this body to sleep,
//	which can be expensive and possibly unintuitive.
fn C.b2Body_SetAwake(bodyId BodyId, awake bool)
@[inline]
pub fn body_set_awake(bodyId BodyId, awake bool) {
	C.b2Body_SetAwake(bodyId, awake)
}

// Enable or disable sleeping for this body. If sleeping is disabled the body will wake.
fn C.b2Body_EnableSleep(bodyId BodyId, enableSleep bool)
@[inline]
pub fn body_enable_sleep(bodyId BodyId, enableSleep bool) {
	C.b2Body_EnableSleep(bodyId, enableSleep)
}

// Returns true if sleeping is enabled for this body
fn C.b2Body_IsSleepEnabled(bodyId BodyId) bool
@[inline]
pub fn body_is_sleep_enabled(bodyId BodyId) bool {
	return C.b2Body_IsSleepEnabled(bodyId)
}

// Set the sleep threshold, typically in meters per second
fn C.b2Body_SetSleepThreshold(bodyId BodyId, sleepVelocity f32)
@[inline]
pub fn body_set_sleep_threshold(bodyId BodyId, sleepVelocity f32) {
	C.b2Body_SetSleepThreshold(bodyId, sleepVelocity)
}

// Get the sleep threshold, typically in meters per second.
fn C.b2Body_GetSleepThreshold(bodyId BodyId) f32
@[inline]
pub fn body_get_sleep_threshold(bodyId BodyId) f32 {
	return C.b2Body_GetSleepThreshold(bodyId)
}

// Returns true if this body is enabled
fn C.b2Body_IsEnabled(bodyId BodyId) bool
@[inline]
pub fn body_is_enabled(bodyId BodyId) bool {
	return C.b2Body_IsEnabled(bodyId)
}

// Disable a body by removing it completely from the simulation. This is expensive.
fn C.b2Body_Disable(bodyId BodyId)
@[inline]
pub fn body_disable(bodyId BodyId) {
	C.b2Body_Disable(bodyId)
}

// Enable a body by adding it to the simulation. This is expensive.
fn C.b2Body_Enable(bodyId BodyId)
@[inline]
pub fn body_enable(bodyId BodyId) {
	C.b2Body_Enable(bodyId)
}

// Set this body to have fixed rotation. This causes the mass to be reset in all cases.
fn C.b2Body_SetFixedRotation(bodyId BodyId, flag bool)  
@[inline]
pub fn body_set_fixed_rotation(bodyId BodyId, flag bool)   {
	C.b2Body_SetFixedRotation(bodyId, flag)  
}

// Does this body have fixed rotation? 
fn C.b2Body_IsFixedRotation(bodyId BodyId) bool
@[inline]
pub fn body_is_fixed_rotation(bodyId BodyId) bool {
	return C.b2Body_IsFixedRotation(bodyId)
}

// Set this body to be a bullet. A bullet does continuous collision detection
// against dynamic bodies (but not other bullets).
fn C.b2Body_SetBullet(bodyId BodyId, flag bool)
@[inline]
pub fn body_set_bullet(bodyId BodyId, flag bool) {
	C.b2Body_SetBullet(bodyId, flag)
}

// Is this body a bullet?
fn C.b2Body_IsBullet(bodyId BodyId) bool
@[inline]
pub fn body_is_bullet(bodyId BodyId) bool {
	return C.b2Body_IsBullet(bodyId) 
}

// Enable/disable hit events on all shapes
//	@see b2ShapeDef::enableHitEvents
fn C.b2Body_EnableHitEvents(bodyId BodyId, enableHitEvents bool)
@[inline]
pub fn body_enable_hit_events(bodyId BodyId, enableHitEvents bool) {
	C.b2Body_EnableHitEvents(bodyId, enableHitEvents)
}

// Get the number of shapes on this body
fn C.b2Body_GetShapeCount(bodyId BodyId) int
@[inline]
pub fn body_get_shape_count(bodyId BodyId) int {
	return C.b2Body_GetShapeCount(bodyId)
}

// Get the shape ids for all shapes on this body, up to the provided capacity.
//	@returns the number of shape ids stored in the user array
fn C.b2Body_GetShapes(bodyId BodyId, shapeArray &ShapeId, capacity int) int
@[inline]
pub fn body_get_shapes(bodyId BodyId, shapeArray &ShapeId, capacity int) int {
	return C.b2Body_GetShapes(bodyId, shapeArray, capacity)
}

// Get the number of joints on this body
fn C.b2Body_GetJointCount(bodyId BodyId) int
@[inline]
pub fn body_get_joint_count(bodyId BodyId) int {
	return C.b2Body_GetJointCount(bodyId)
}

// Get the joint ids for all joints on this body, up to the provided capacity
//	@returns the number of joint ids stored in the user array
fn C.b2Body_GetJoints(bodyId BodyId, jointArray &JointId, capacity int) int
@[inline]
pub fn body_get_joints(bodyId BodyId, jointArray &JointId, capacity int) int {
	return C.b2Body_GetJoints(bodyId, jointArray, capacity)
}

// Get the maximum capacity required for retrieving all the touching contacts on a body
fn C.b2Body_GetContactCapacity(bodyId BodyId) int
@[inline]
pub fn body_get_contact_capacity(bodyId BodyId) int {
	return C.b2Body_GetContactCapacity(bodyId)
}

// Get the touching contact data for a body
fn C.b2Body_GetContactData(bodyId BodyId, contactData &ContactData, capacity int) int
@[inline]
pub fn body_get_contact_data(bodyId BodyId, contactData &ContactData, capacity int) int {
	return C.b2Body_GetContactData(bodyId, contactData, capacity)
}

// Get the current world AABB that contains all the attached shapes. Note that this may not encompass the body origin.
//	If there are no shapes attached then the returned AABB is empty and centered on the body origin.
fn C.b2Body_ComputeAABB(bodyId BodyId) AABB 
@[inline]
pub fn body_compute_aabb(bodyId BodyId) AABB  {
	return C.b2Body_ComputeAABB(bodyId) 
}

/** @} */

/**
 * @defgroup shape Shape
 * Functions to create, destroy, and access.
 * Shapes bind raw geometry to bodies and hold material properties including friction and restitution.
 * @{
 */

// Create a circle shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
//	@return the shape id for accessing the shape
fn C.b2CreateCircleShape(bodyId BodyId, def &ShapeDef, circle &Circle) ShapeId 
@[inline]
pub fn create_circle_shape(bodyId BodyId, def &ShapeDef, circle &Circle) ShapeId  {
	return C.b2CreateCircleShape(bodyId, def, circle) 
}

// Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
//	@return the shape id for accessing the shape
fn C.b2CreateSegmentShape(bodyId BodyId, def &ShapeDef, segment &Segment) ShapeId
@[inline]
pub fn create_segment_shape(bodyId BodyId, def &ShapeDef, segment &Segment) ShapeId {
	return C.b2CreateSegmentShape(bodyId, def, segment)
}

// Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
//	@return the shape id for accessing the shape
fn C.b2CreateCapsuleShape(bodyId BodyId, def &ShapeDef, capsule &Capsule) ShapeId
@[inline]
pub fn create_capsule_shape(bodyId BodyId, def &ShapeDef, capsule &Capsule) ShapeId {
	return C.b2CreateCapsuleShape(bodyId, def, capsule)
}

// Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
// Contacts are not created until the next time step.
//	@return the shape id for accessing the shape
fn C.b2CreatePolygonShape(bodyId BodyId, def &ShapeDef, polygon &Polygon) ShapeId
@[inline]
pub fn create_polygon_shape(bodyId BodyId, def &ShapeDef, polygon &Polygon) ShapeId {
	return C.b2CreatePolygonShape(bodyId, def, polygon)
}

// Destroy a shape
fn C.b2DestroyShape(shapeId ShapeId)
@[inline]
pub fn destroy_shape(shapeId ShapeId) {
	C.b2DestroyShape(shapeId)
}

// Shape identifier validation. Provides validation for up to 64K allocations.
fn C.b2Shape_IsValid(id ShapeId) bool
@[inline]
pub fn shape_is_valid(id ShapeId) bool {
	return C.b2Shape_IsValid(id)
}

// Get the type of a shape
fn C.b2Shape_GetType(shapeId ShapeId) ShapeType
@[inline]
pub fn shape_get_type(shapeId ShapeId) ShapeType {
 	return C.b2Shape_GetType(shapeId) 
}

// Get the id of the body that a shape is attached to
fn C.b2Shape_GetBody(shapeId ShapeId) BodyId
@[inline]
pub fn shape_get_body(shapeId ShapeId) BodyId {
	return C.b2Shape_GetBody(shapeId)
}

// Returns true If the shape is a sensor
fn C.b2Shape_IsSensor(shapeId ShapeId) bool
@[inline]
pub fn shape_is_sensor(shapeId ShapeId) bool {
	return C.b2Shape_IsSensor(shapeId)
}

// Set the user data for a shape
fn C.b2Shape_SetUserData(shapeId ShapeId, userData voidptr)
@[inline]
pub fn shape_set_user_data(shapeId ShapeId, userData voidptr) {
	C.b2Shape_SetUserData(shapeId, userData)
}

// Get the user data for a shape. This is useful when you get a shape id
//	from an event or query.
fn C.b2Shape_GetUserData(shapeId ShapeId) voidptr
@[inline]
pub fn shape_get_user_data(shapeId ShapeId) voidptr {
	return C.b2Shape_GetUserData(shapeId)
}

// Set the mass density of a shape, typically in kg/m^2.
//	This will not update the mass properties on the parent body.
//	@see b2ShapeDef::density, b2Body_ApplyMassFromShapes
fn C.b2Shape_SetDensity(shapeId ShapeId, density f32)
@[inline]
pub fn shape_set_density(shapeId ShapeId, density f32) {
	C.b2Shape_SetDensity(shapeId, density)
}

// Get the density of a shape, typically in kg/m^2
fn C.b2Shape_GetDensity(shapeId ShapeId) f32
@[inline]
pub fn shape_get_density(shapeId ShapeId) f32 {
	return C.b2Shape_GetDensity(shapeId)
}

// Set the friction on a shape
//	@see b2ShapeDef::friction
fn C.b2Shape_SetFriction(shapeId ShapeId, friction f32)
@[inline]
pub fn shape_set_friction(shapeId ShapeId, friction f32) {
	C.b2Shape_SetFriction(shapeId, friction)
}

// Get the friction of a shape
fn C.b2Shape_GetFriction(shapeId ShapeId) f32
@[inline]
pub fn shape_get_friction(shapeId ShapeId) f32 {
	return C.b2Shape_GetFriction(shapeId)
}

// Set the shape restitution (bounciness)
//	@see b2ShapeDef::restitution
fn C.b2Shape_SetRestitution(shapeId ShapeId, restitution f32)
@[inline]
pub fn shape_set_restitution(shapeId ShapeId, restitution f32) {
	C.b2Shape_SetRestitution(shapeId, restitution)
}

// Get the shape restitution
fn C.b2Shape_GetRestitution(shapeId ShapeId) f32
@[inline]
pub fn shape_get_restitution(shapeId ShapeId) f32 {
	return C.b2Shape_GetRestitution(shapeId)
}

// Get the shape filter
fn C.b2Shape_GetFilter(shapeId ShapeId) Filter
@[inline]
pub fn shape_get_filter(shapeId ShapeId) Filter {
	return C.b2Shape_GetFilter(shapeId)
}

// Set the current filter. This is almost as expensive as recreating the shape.
//	@see b2ShapeDef::filter
fn C.b2Shape_SetFilter(shapeId ShapeId, filter Filter)
@[inline]
pub fn shape_set_filter(shapeId ShapeId, filter Filter) {
	C.b2Shape_SetFilter(shapeId, filter)
}

// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
//	@see b2ShapeDef::isSensor
fn C.b2Shape_EnableSensorEvents(shapeId ShapeId, flag bool)
@[inline]
pub fn shape_enable_sensor_events(shapeId ShapeId, flag bool) {
	C.b2Shape_EnableSensorEvents(shapeId, flag)
}

// Returns true if sensor events are enabled
fn C.b2Shape_AreSensorEventsEnabled(shapeId ShapeId) bool
@[inline]
pub fn shape_are_sensor_events_enabled(shapeId ShapeId) bool {
	return C.b2Shape_AreSensorEventsEnabled(shapeId)
}

// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
//	@see b2ShapeDef::enableContactEvents
fn C.b2Shape_EnableContactEvents(shapeId ShapeId, flag bool)
@[inline]
pub fn shape_enable_contact_events(shapeId ShapeId, flag bool) {
	C.b2Shape_EnableContactEvents(shapeId, flag)
}

// Returns true if contact events are enabled
fn C.b2Shape_AreContactEventsEnabled(shapeId ShapeId) bool
@[inline]
pub fn shape_are_contact_events_enabled(shapeId ShapeId) bool {
	return C.b2Shape_AreContactEventsEnabled(shapeId)
}

// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
//	and must be carefully handled due to multithreading. Ignored for sensors.
//	@see b2PreSolveFcn
fn C.b2Shape_EnablePreSolveEvents(shapeId ShapeId, flag bool)
@[inline]
pub fn shape_enable_pre_solve_events(shapeId ShapeId, flag bool) {
	C.b2Shape_EnablePreSolveEvents(shapeId, flag)
}

// Returns true if pre-solve events are enabled
fn C.b2Shape_ArePreSolveEventsEnabled(shapeId ShapeId) bool
@[inline]
pub fn shape_are_pre_solve_events_enabled(shapeId ShapeId) bool {
	return C.b2Shape_ArePreSolveEventsEnabled(shapeId)
}

// Enable contact hit events for this shape. Ignored for sensors.
//	@see b2WorldDef.hitEventThreshold
fn C.b2Shape_EnableHitEvents(shapeId ShapeId, flag bool)
@[inline]
pub fn shape_enable_hit_events(shapeId ShapeId, flag bool) {
	C.b2Shape_EnableHitEvents(shapeId, flag)
}

// Returns true if hit events are enabled
fn C.b2Shape_AreHitEventsEnabled(shapeId ShapeId) bool
@[inline]
pub fn shape_are_hit_events_enabled(shapeId ShapeId) bool {
	return C.b2Shape_AreHitEventsEnabled(shapeId)
}

// Test a point for overlap with a shape
fn C.b2Shape_TestPoint(shapeId ShapeId, point Vec2) bool
@[inline]
pub fn shape_test_point(shapeId ShapeId, point Vec2) bool {
	return C.b2Shape_TestPoint(shapeId, point)
}

// Ray cast a shape directly
fn C.b2Shape_RayCast(shapeId ShapeId, origin Vec2, translation Vec2) CastOutput
@[inline]
pub fn shape_ray_cast(shapeId ShapeId, origin Vec2, translation Vec2) CastOutput {
	return C.b2Shape_RayCast(shapeId, origin, translation)
}

// Get a copy of the shape's circle. Asserts the type is correct.
fn C.b2Shape_GetCircle(shapeId ShapeId) Circle
@[inline]
pub fn shape_get_circle(shapeId ShapeId) Circle {
	return C.b2Shape_GetCircle(shapeId)
}

// Get a copy of the shape's line segment. Asserts the type is correct.
fn C.b2Shape_GetSegment(shapeId ShapeId) Segment 
@[inline]
pub fn shape_get_segment(shapeId ShapeId) Segment  {
	return C.b2Shape_GetSegment(shapeId)
}

// Get a copy of the shape's smooth line segment. These come from chain shapes.
// Asserts the type is correct.
fn C.b2Shape_GetSmoothSegment(shapeId ShapeId) SmoothSegment 
@[inline]
pub fn shape_get_smooth_segment(shapeId ShapeId) SmoothSegment  {
	return C.b2Shape_GetSmoothSegment(shapeId)
}

// Get a copy of the shape's capsule. Asserts the type is correct.
fn C.b2Shape_GetCapsule(shapeId ShapeId) Capsule
@[inline]
pub fn shape_get_capsule(shapeId ShapeId) Capsule {
	return C.b2Shape_GetCapsule(shapeId)
}

// Get a copy of the shape's convex polygon. Asserts the type is correct.
fn C.b2Shape_GetPolygon(shapeId ShapeId) Polygon
@[inline]
pub fn shape_get_polygon(shapeId ShapeId) Polygon {
	return C.b2Shape_GetPolygon(shapeId)
}

// Allows you to change a shape to be a circle or update the current circle.
// This does not modify the mass properties.
//	@see b2Body_ApplyMassFromShapes
fn C.b2Shape_SetCircle(shapeId ShapeId, circle &Circle)
@[inline]
pub fn shape_set_circle(shapeId ShapeId, circle &Circle) {
	C.b2Shape_SetCircle(shapeId, circle)
}

// Allows you to change a shape to be a capsule or update the current capsule.
// This does not modify the mass properties.
//	@see b2Body_ApplyMassFromShapes
fn C.b2Shape_SetCapsule(shapeId ShapeId, capsule &Capsule)
@[inline]
pub fn shape_set_capsule(shapeId ShapeId, capsule &Capsule) {
	C.b2Shape_SetCapsule(shapeId, capsule)
}

// Allows you to change a shape to be a segment or update the current segment.
fn C.b2Shape_SetSegment(shapeId ShapeId, segment &Segment)
@[inline]
pub fn shape_set_segment(shapeId ShapeId, segment &Segment) {
	C.b2Shape_SetSegment(shapeId, segment)
}

// Allows you to change a shape to be a polygon or update the current polygon.
// This does not modify the mass properties.
//	@see b2Body_ApplyMassFromShapes
fn C.b2Shape_SetPolygon(shapeId ShapeId, polygon &Polygon)
@[inline]
pub fn shape_set_polygon(shapeId ShapeId, polygon &Polygon) {
	C.b2Shape_SetPolygon(shapeId, polygon)
}

// Get the parent chain id if the shape type is smoothSegmentShape, otherwise
// returns nullChainId.
fn C.b2Shape_GetParentChain(shapeId ShapeId) ChainId
@[inline]
pub fn shape_get_parent_chain(shapeId ShapeId) ChainId {
	return C.b2Shape_GetParentChain(shapeId)
}

// Get the maximum capacity required for retrieving all the touching contacts on a shape
fn C.b2Shape_GetContactCapacity(shapeId ShapeId) int
@[inline]
pub fn shape_get_contact_capacity(shapeId ShapeId) int {
	return C.b2Shape_GetContactCapacity(shapeId)
}

// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
fn C.b2Shape_GetContactData(shapeId ShapeId, contactData &ContactData, capacity int) int
@[inline]
pub fn shape_get_contact_data(shapeId ShapeId, contactData &ContactData, capacity int) int {
	return C.b2Shape_GetContactData(shapeId, contactData, capacity)
}

// Get the current world AABB
fn C.b2Shape_GetAABB(shapeId ShapeId) AABB
@[inline]
pub fn shape_get_aabb(shapeId ShapeId) AABB {
	return C.b2Shape_GetAABB(shapeId)
}

// Get the closest point on a shape to a target point. Target and result are in world space.
fn C.b2Shape_GetClosestPoint(shapeId ShapeId, target Vec2) Vec2
@[inline]
pub fn shape_get_closest_point(shapeId ShapeId, target Vec2) Vec2 {
	return C.b2Shape_GetClosestPoint(shapeId, target)
}

// Chain Shape

// Create a chain shape
//	@see b2ChainDef for details
fn C.b2CreateChain(bodyId BodyId, def &ChainDef) ChainId
@[inline]
pub fn create_chain(bodyId BodyId, def &ChainDef) ChainId {
	return C.b2CreateChain(bodyId, def)
}

// Destroy a chain shape
fn C.b2DestroyChain(chainId ChainId)
@[inline]
pub fn destroy_chain(chainId ChainId) {
	C.b2DestroyChain(chainId)
}

// Set the chain friction
// @see b2ChainDef::friction
fn C.b2Chain_SetFriction(chainId ChainId, friction f32)
@[inline]
pub fn chain_set_friction(chainId ChainId, friction f32) {
	C.b2Chain_SetFriction(chainId, friction)
}

// Set the chain restitution (bounciness)
// @see b2ChainDef::restitution
fn C.b2Chain_SetRestitution(chainId ChainId, restitution f32)
@[inline]
pub fn chain_set_restitution(chainId ChainId, restitution f32) {
	C.b2Chain_SetRestitution(chainId, restitution)
}

// Chain identifier validation. Provides validation for up to 64K allocations.
fn C.b2Chain_IsValid(id ChainId) bool
@[inline]
pub fn chain_is_valid(id ChainId) bool {
	return C.b2Chain_IsValid(id)
}

/** @} */

/**
 * @defgroup joint Joint
 * @brief Joints allow you to connect rigid bodies together while allowing various forms of relative motions.
 * @{
 */

// Destroy a joint
fn C.b2DestroyJoint(jointId JointId)
@[inline]
pub fn destroy_joint(jointId JointId) {
	C.b2DestroyJoint(jointId)
}

// Joint identifier validation. Provides validation for up to 64K allocations.
fn C.b2Joint_IsValid(id JointId) bool
@[inline]
pub fn joint_is_valid(id JointId) bool {
	return C.b2Joint_IsValid(id)
}

// Get the joint type
fn C.b2Joint_GetType(jointId JointId) JointType
@[inline]
pub fn joint_get_type(jointId JointId) JointType {
	return C.b2Joint_GetType(jointId)
}

// Get body A id on a joint
fn C.b2Joint_GetBodyA(jointId JointId) BodyId
@[inline]
pub fn joint_get_body_a(jointId JointId) BodyId {
	return C.b2Joint_GetBodyA(jointId)
}

// Get body B id on a joint
fn C.b2Joint_GetBodyB(jointId JointId) BodyId
@[inline]
pub fn joint_get_body_b(jointId JointId) BodyId {
	return C.b2Joint_GetBodyB(jointId)
}

// Get the local anchor on bodyA
fn C.b2Joint_GetLocalAnchorA(jointId JointId) Vec2
@[inline]
pub fn joint_get_local_anchor_a(jointId JointId) Vec2 {
	return C.b2Joint_GetLocalAnchorA(jointId)
}

// Get the local anchor on bodyB
fn C.b2Joint_GetLocalAnchorB(jointId JointId) Vec2
@[inline]
pub fn joint_get_local_anchor_b(jointId JointId) Vec2 {
	return C.b2Joint_GetLocalAnchorB(jointId)
}

// Toggle collision between connected bodies
fn C.b2Joint_SetCollideConnected(jointId JointId, shouldCollide bool)
@[inline]
pub fn joint_set_collide_connected(jointId JointId, shouldCollide bool) {
	C.b2Joint_SetCollideConnected(jointId, shouldCollide)
}

// Is collision allowed between connected bodies?
fn C.b2Joint_GetCollideConnected(jointId JointId) bool
@[inline]
pub fn joint_get_collide_connected(jointId JointId) bool {
	return C.b2Joint_GetCollideConnected(jointId)
}

// Set the user data on a joint
fn C.b2Joint_SetUserData(jointId JointId, userData voidptr)
@[inline]
pub fn joint_set_user_data(jointId JointId, userData voidptr) {
	C.b2Joint_SetUserData(jointId, userData)
}

// Get the user data on a joint
fn C.b2Joint_GetUserData(jointId JointId) voidptr
@[inline]
pub fn joint_get_user_data(jointId JointId) voidptr {
	return C.b2Joint_GetUserData(jointId)
}

// Wake the bodies connect to this joint
fn C.b2Joint_WakeBodies(jointId JointId)
@[inline]
pub fn joint_wake_bodies(jointId JointId) {
	C.b2Joint_WakeBodies(jointId)
}

// Get the current constraint force for this joint
fn C.b2Joint_GetConstraintForce(jointId JointId) Vec2
@[inline]
pub fn joint_get_constraint_force(jointId JointId) Vec2 {
	return C.b2Joint_GetConstraintForce(jointId)
}

// Get the current constraint torque for this joint
fn C.b2Joint_GetConstraintTorque(jointId JointId) f32
@[inline]
pub fn joint_get_constraint_torque(jointId JointId) f32 {
	return C.b2Joint_GetConstraintTorque(jointId)
}

/**
 * @defgroup distance_joint Distance Joint
 * @brief Functions for the distance joint.
 * @{
 */

// Create a distance joint
//	@see b2DistanceJointDef for details
fn C.b2CreateDistanceJoint(worldId WorldId, def &DistanceJointDef) JointId 
@[inline]
pub fn create_distance_joint(worldId WorldId, def &DistanceJointDef) JointId  {
	return C.b2CreateDistanceJoint(worldId, def)
}

// Set the rest length of a distance joint
// @param jointId The id for a distance joint
// @param length The new distance joint length
fn C.b2DistanceJoint_SetLength(jointId JointId, length f32)
@[inline]
pub fn distance_joint_set_length(jointId JointId, length f32) {
	C.b2DistanceJoint_SetLength(jointId, length)
}

// Get the rest length of a distance joint
fn C.b2DistanceJoint_GetLength(jointId JointId) f32
@[inline]
pub fn distance_joint_get_length(jointId JointId) f32 {
	return C.b2DistanceJoint_GetLength(jointId)
}

// Enable/disable the distance joint spring. When disabled the distance joint is rigid.
fn C.b2DistanceJoint_EnableSpring(jointId JointId, enableSpring bool)
@[inline]
pub fn distance_joint_enable_spring(jointId JointId, enableSpring bool) {
	C.b2DistanceJoint_EnableSpring(jointId, enableSpring)
}

// Is the distance joint spring enabled?
fn C.b2DistanceJoint_IsSpringEnabled(jointId JointId) bool
@[inline]
pub fn distance_joint_is_spring_enabled(jointId JointId) bool {
	return C.b2DistanceJoint_IsSpringEnabled(jointId)
}

// Set the spring stiffness in Hertz
fn C.b2DistanceJoint_SetSpringHertz(jointId JointId, hertz f32)
@[inline]
pub fn distance_joint_set_spring_hertz(jointId JointId, hertz f32) {
	C.b2DistanceJoint_SetSpringHertz(jointId, hertz)
}

// Set the spring damping ratio, non-dimensional
fn C.b2DistanceJoint_SetSpringDampingRatio(jointId JointId, dampingRatio f32)
@[inline]
pub fn distance_joint_set_spring_damping_ratio(jointId JointId, dampingRatio f32) {
	C.b2DistanceJoint_SetSpringDampingRatio(jointId, dampingRatio)
}

// Get the spring Hertz
fn C.b2DistanceJoint_GetHertz(jointId JointId) f32
@[inline]
pub fn distance_joint_get_hertz(jointId JointId) f32 {
	return C.b2DistanceJoint_GetHertz(jointId)
}

// Get the spring damping ratio
fn C.b2DistanceJoint_GetDampingRatio(jointId JointId) f32
@[inline]
pub fn distance_joint_get_damping_ratio(jointId JointId) f32 {
	return C.b2DistanceJoint_GetDampingRatio(jointId)
}

// Enable joint limit. The limit only works if the joint spring is enabled. Otherwise the joint is rigid
//	and the limit has no effect.
fn C.b2DistanceJoint_EnableLimit(jointId JointId, enableLimit bool)
@[inline]
pub fn distance_joint_enable_limit(jointId JointId, enableLimit bool) {
	C.b2DistanceJoint_EnableLimit(jointId, enableLimit)
}

// Is the distance joint limit enabled?
fn C.b2DistanceJoint_IsLimitEnabled(jointId JointId) bool
@[inline]
pub fn distance_joint_is_limit_enabled(jointId JointId) bool {
	return C.b2DistanceJoint_IsLimitEnabled(jointId)
}

// Set the minimum and maximum length parameters of a distance joint
fn C.b2DistanceJoint_SetLengthRange(jointId JointId, minLength f32, maxLength f32)
@[inline]
pub fn distance_joint_set_length_range(jointId JointId, minLength f32, maxLength f32) {
	C.b2DistanceJoint_SetLengthRange(jointId, minLength, maxLength)
}

// Get the distance joint minimum length
fn C.b2DistanceJoint_GetMinLength(jointId JointId) f32
@[inline]
pub fn distance_joint_get_min_length(jointId JointId) f32 {
	return C.b2DistanceJoint_GetMinLength(jointId)
}

// Get the distance joint maximum length
fn C.b2DistanceJoint_GetMaxLength(jointId JointId) f32
@[inline]
pub fn distance_joint_get_max_length(jointId JointId) f32 {
	return C.b2DistanceJoint_GetMaxLength(jointId)
}

// Get the current length of a distance joint
fn C.b2DistanceJoint_GetCurrentLength(jointId JointId) f32
@[inline]
pub fn distance_joint_get_current_length(jointId JointId) f32 {
	return C.b2DistanceJoint_GetCurrentLength(jointId) 
}

// Enable/disable the distance joint motor
fn C.b2DistanceJoint_EnableMotor(jointId JointId, enableMotor bool)
@[inline]
pub fn distance_joint_enable_motor(jointId JointId, enableMotor bool) {
	C.b2DistanceJoint_EnableMotor(jointId, enableMotor)
}

// Is the distance joint motor enabled?
fn C.b2DistanceJoint_IsMotorEnabled(jointId JointId) bool
@[inline]
pub fn distance_joint_is_motor_enabled(jointId JointId) bool {
	return C.b2DistanceJoint_IsMotorEnabled(jointId)
}

// Set the distance joint motor speed, typically in meters per second
fn C.b2DistanceJoint_SetMotorSpeed(jointId JointId, motorSpeed f32)
@[inline]
pub fn distance_joint_set_motor_speed(jointId JointId, motorSpeed f32) {
	C.b2DistanceJoint_SetMotorSpeed(jointId, motorSpeed)
}

// Get the distance joint motor speed, typically in meters per second
fn C.b2DistanceJoint_GetMotorSpeed(jointId JointId) f32
@[inline]
pub fn distance_joint_get_motor_speed(jointId JointId) f32 {
	return C.b2DistanceJoint_GetMotorSpeed(jointId)
}

// Set the distance joint maximum motor force, typically in newtons
fn C.b2DistanceJoint_SetMaxMotorForce(jointId JointId, force f32)
@[inline]
pub fn distance_joint_set_max_motor_force(jointId JointId, force f32) {
	C.b2DistanceJoint_SetMaxMotorForce(jointId, force)
}

// Get the distance joint maximum motor force, typically in newtons
fn C.b2DistanceJoint_GetMaxMotorForce(jointId JointId) f32
@[inline]
pub fn distance_joint_get_max_motor_force(jointId JointId) f32 {
	return C.b2DistanceJoint_GetMaxMotorForce(jointId)
}

// Get the distance joint current motor force, typically in newtons
fn C.b2DistanceJoint_GetMotorForce(jointId JointId) f32
@[inline]
pub fn distance_joint_get_motor_force(jointId JointId) f32 {
	return C.b2DistanceJoint_GetMotorForce(jointId)
}

/** @} */

/**
 * @defgroup motor_joint Motor Joint
 * @brief Functions for the motor joint.
 *
 * The motor joint is used to drive the relative transform between two bodies. It takes
 * a relative position and rotation and applies the forces and torques needed to achieve
 * that relative transform over time.
 * @{
 */

// Create a motor joint
//	@see b2MotorJointDef for details
fn C.b2CreateMotorJoint(worldId WorldId, def &MotorJointDef) JointId
@[inline]
pub fn create_motor_joint(worldId WorldId, def &MotorJointDef) JointId {
	return C.b2CreateMotorJoint(worldId, def)
}

// Set the motor joint linear offset target
fn C.b2MotorJoint_SetLinearOffset(jointId JointId, linearOffset Vec2)
@[inline]
pub fn motor_joint_set_linear_offset(jointId JointId, linearOffset Vec2) {
	C.b2MotorJoint_SetLinearOffset(jointId, linearOffset)
}

// Get the motor joint linear offset target
fn C.b2MotorJoint_GetLinearOffset(jointId JointId) Vec2
@[inline]
pub fn motor_joint_get_linear_offset(jointId JointId) Vec2 {
	return C.b2MotorJoint_GetLinearOffset(jointId)
}

// Set the motor joint angular offset target in radians
fn C.b2MotorJoint_SetAngularOffset(jointId JointId, angularOffset f32)
@[inline]
pub fn motor_joint_set_angular_offset(jointId JointId, angularOffset f32) {
	C.b2MotorJoint_SetAngularOffset(jointId, angularOffset)
}

// Get the motor joint angular offset target in radians
fn C.b2MotorJoint_GetAngularOffset(jointId JointId) f32
@[inline]
pub fn motor_joint_get_angular_offset(jointId JointId) f32 {
	return C.b2MotorJoint_GetAngularOffset(jointId) 
}

// Set the motor joint maximum force, typically in newtons
fn C.b2MotorJoint_SetMaxForce(jointId JointId, maxForce f32)
@[inline]
pub fn motor_joint_set_max_force(jointId JointId, maxForce f32) {
	C.b2MotorJoint_SetMaxForce(jointId, maxForce)
}

// Get the motor joint maximum force, typically in newtons
fn C.b2MotorJoint_GetMaxForce(jointId JointId) f32
@[inline]
pub fn motor_joint_get_max_force(jointId JointId) f32 {
	return C.b2MotorJoint_GetMaxForce(jointId)
}

// Set the motor joint maximum torque, typically in newton-meters
fn C.b2MotorJoint_SetMaxTorque(jointId JointId, maxTorque f32)
@[inline]
pub fn motor_joint_set_max_torque(jointId JointId, maxTorque f32) {
	C.b2MotorJoint_SetMaxTorque(jointId, maxTorque)
}

// Get the motor joint maximum torque, typically in newton-meters
fn C.b2MotorJoint_GetMaxTorque(jointId JointId) f32
@[inline]
pub fn motor_joint_get_max_torque(jointId JointId) f32 {
	return C.b2MotorJoint_GetMaxTorque(jointId)
}

// Set the motor joint correction factor, typically in [0, 1]
fn C.b2MotorJoint_SetCorrectionFactor(jointId JointId, correctionFactor f32)
@[inline]
pub fn motor_joint_set_correction_factor(jointId JointId, correctionFactor f32) {
	C.b2MotorJoint_SetCorrectionFactor(jointId, correctionFactor)
}

// Get the motor joint correction factor, typically in [0, 1]
fn C.b2MotorJoint_GetCorrectionFactor(jointId JointId) f32
@[inline]
pub fn motor_joint_get_correction_factor(jointId JointId) f32  {
	return C.b2MotorJoint_GetCorrectionFactor(jointId) 
}

/**@}*/

/**
 * @defgroup mouse_joint Mouse Joint
 * @brief Functions for the mouse joint.
 * 
 * The mouse joint is designed for use in the samples application, but you may find it useful in applications where
 * the user moves a rigid body with a cursor.
 * @{
 */

// Create a mouse joint
//	@see b2MouseJointDef for details
fn C.b2CreateMouseJoint(worldId WorldId, def &MouseJointDef) JointId
@[inline]
pub fn create_mouse_joint(worldId WorldId, def &MouseJointDef) JointId {
	return C.b2CreateMouseJoint(worldId, def)
}

// Set the mouse joint target
fn C.b2MouseJoint_SetTarget(jointId JointId, target Vec2)
@[inline]
pub fn mouse_joint_set_target(jointId JointId, target Vec2)  {
	C.b2MouseJoint_SetTarget(jointId, target)
}

// Get the mouse joint target
fn C.b2MouseJoint_GetTarget(jointId JointId) Vec2
@[inline]
pub fn mouse_joint_get_target(jointId JointId) Vec2 {
	return C.b2MouseJoint_GetTarget(jointId)
}

// Set the mouse joint spring stiffness in Hertz
fn C.b2MouseJoint_SetSpringHertz(jointId JointId, hertz f32)
@[inline]
pub fn mouse_joint_set_spring_hertz(jointId JointId, hertz f32) {
	C.b2MouseJoint_SetSpringHertz(jointId, hertz)
}

// Get the mouse joint spring stiffness in Hertz
fn C.b2MouseJoint_GetSpringHertz(jointId JointId) f32
@[inline]
pub fn mouse_joint_get_spring_hertz(jointId JointId) f32 {
	return C.b2MouseJoint_GetSpringHertz(jointId)
}

// Set the mouse joint spring damping ratio, non-dimensional
fn C.b2MouseJoint_SetSpringDampingRatio(jointId JointId, dampingRatio f32)
@[inline]
pub fn mouse_joint_set_spring_damping_ratio(jointId JointId, dampingRatio f32) {
	C.b2MouseJoint_SetSpringDampingRatio(jointId, dampingRatio)
}

// Get the mouse joint damping ratio, non-dimensional
fn C.b2MouseJoint_GetSpringDampingRatio(jointId JointId) f32
@[inline]
pub fn mouse_joint_get_spring_damping_ratio(jointId JointId) f32 {
	return C.b2MouseJoint_GetSpringDampingRatio(jointId)
}

// Set the mouse joint maximum force, typically in newtons
fn C.b2MouseJoint_SetMaxForce(jointId JointId, maxForce f32)
@[inline]
pub fn mouse_joint_set_max_force(jointId JointId, maxForce f32) {
	C.b2MouseJoint_SetMaxForce(jointId, maxForce)
}

// Get the mouse joint maximum force, typically in newtons
fn C.b2MouseJoint_GetMaxForce(jointId JointId) f32
@[inline]
pub fn mouse_joint_get_max_force(jointId JointId) f32 {
	return C.b2MouseJoint_GetMaxForce(jointId) 
}

/**@}*/

/**
 * @defgroup prismatic_joint Prismatic Joint
 * @brief A prismatic joint allows for translation along a single axis with no rotation.
 * 
 * The prismatic joint is useful for things like pistons and moving platforms, where you want a body to translate
 * along an axis and have no rotation. Also called a *slider* joint.
 * @{
 */

// Create a prismatic (slider) joint.
//	@see b2PrismaticJointDef for details
fn C.b2CreatePrismaticJoint(worldId WorldId, def &PrismaticJointDef) JointId
@[inline]
pub fn create_prismatic_joint(worldId WorldId, def &PrismaticJointDef) JointId {
	return C.b2CreatePrismaticJoint(worldId, def) 
}

// Enable/disable the joint spring.
fn C.b2PrismaticJoint_EnableSpring(jointId JointId, enableSpring bool)
@[inline]
pub fn prismatic_joint_enable_spring(jointId JointId, enableSpring bool) {
	C.b2PrismaticJoint_EnableSpring(jointId, enableSpring)
}

// Is the prismatic joint spring enabled or not?
fn C.b2PrismaticJoint_IsSpringEnabled(jointId JointId) bool
@[inline]
pub fn prismatic_joint_is_spring_enabled(jointId JointId) bool {
	return C.b2PrismaticJoint_IsSpringEnabled(jointId)
}

// Set the prismatic joint stiffness in Hertz.
// This should usually be less than a quarter of the simulation rate. For example, if the simulation
// runs at 60Hz then the joint stiffness should be 15Hz or less.
fn C.b2PrismaticJoint_SetSpringHertz(jointId JointId, hertz f32)
@[inline]
pub fn prismatic_joint_set_spring_hertz(jointId JointId, hertz f32) {
	C.b2PrismaticJoint_SetSpringHertz(jointId, hertz)
}

// Get the prismatic joint stiffness in Hertz
fn C.b2PrismaticJoint_GetSpringHertz(jointId JointId) f32
@[inline]
pub fn prismatic_joint_get_spring_hertz(jointId JointId) f32 {
	return C.b2PrismaticJoint_GetSpringHertz(jointId)
}

// Set the prismatic joint damping ratio (non-dimensional)
fn C.b2PrismaticJoint_SetSpringDampingRatio(jointId JointId, dampingRatio f32)
@[inline]
pub fn prismatic_joint_set_spring_damping_ratio(jointId JointId, dampingRatio f32) {
	C.b2PrismaticJoint_SetSpringDampingRatio(jointId, dampingRatio)
}

// Get the prismatic spring damping ratio (non-dimensional)
fn C.b2PrismaticJoint_GetSpringDampingRatio(jointId JointId) f32
@[inline]
pub fn prismatic_joint_get_spring_damping_ratio(jointId JointId) f32 {
	return C.b2PrismaticJoint_GetSpringDampingRatio(jointId)
}

// Enable/disable a prismatic joint limit
fn C.b2PrismaticJoint_EnableLimit(jointId JointId, enableLimit bool)
@[inline]
pub fn prismatic_joint_enable_limit(jointId JointId, enableLimit bool) {
	C.b2PrismaticJoint_EnableLimit(jointId, enableLimit)
}

// Is the prismatic joint limit enabled?
fn C.b2PrismaticJoint_IsLimitEnabled(jointId JointId) bool
@[inline]
pub fn prismatic_joint_is_limit_enabled(jointId JointId) bool {
	return C.b2PrismaticJoint_IsLimitEnabled(jointId)
}

// Get the prismatic joint lower limit
fn C.b2PrismaticJoint_GetLowerLimit(jointId JointId) f32
@[inline]
pub fn prismatic_joint_get_lower_limit(jointId JointId) f32 {
	return C.b2PrismaticJoint_GetLowerLimit(jointId)
}

// Get the prismatic joint upper limit
fn C.b2PrismaticJoint_GetUpperLimit(jointId JointId) f32
@[inline]
pub fn prismatic_joint_get_upper_limit(jointId JointId) f32 {
	return C.b2PrismaticJoint_GetUpperLimit(jointId)
}

// Set the prismatic joint limits
fn C.b2PrismaticJoint_SetLimits(jointId JointId, lower f32, upper f32)
@[inline]
pub fn prismatic_joint_set_limits(jointId JointId, lower f32, upper f32) {
	C.b2PrismaticJoint_SetLimits(jointId, lower, upper)
}

// Enable/disable a prismatic joint motor
fn C.b2PrismaticJoint_EnableMotor(jointId JointId, enableMotor bool)
@[inline]
pub fn prismatic_joint_enable_motor(jointId JointId, enableMotor bool) {
	C.b2PrismaticJoint_EnableMotor(jointId, enableMotor)
}

// Is the prismatic joint motor enabled?
fn C.b2PrismaticJoint_IsMotorEnabled(jointId JointId) bool
@[inline]
pub fn prismatic_joint_is_motor_enabled(jointId JointId) bool {
	return C.b2PrismaticJoint_IsMotorEnabled(jointId) 
}

// Set the prismatic joint motor speed, typically in meters per second
fn C.b2PrismaticJoint_SetMotorSpeed(jointId JointId, motorSpeed f32)
@[inline]
pub fn prismatic_joint_set_motor_speed(jointId JointId, motorSpeed f32) {
	C.b2PrismaticJoint_SetMotorSpeed(jointId, motorSpeed)
}

// Get the prismatic joint motor speed, typically in meters per second
fn C.b2PrismaticJoint_GetMotorSpeed(jointId JointId) f32
@[inline]
pub fn prismatic_joint_get_motor_speed(jointId JointId) f32 {
	return C.b2PrismaticJoint_GetMotorSpeed(jointId)
}

// Set the prismatic joint maximum motor force, typically in newtons
fn C.b2PrismaticJoint_SetMaxMotorForce(jointId JointId, force f32)
@[inline]
pub fn prismatic_joint_set_max_motor_force(jointId JointId, force f32) {
	C.b2PrismaticJoint_SetMaxMotorForce(jointId, force)
}

// Get the prismatic joint maximum motor force, typically in newtons
fn C.b2PrismaticJoint_GetMaxMotorForce(jointId JointId) f32
@[inline]
pub fn prismatic_joint_get_max_motor_force(jointId JointId) f32 {
	return C.b2PrismaticJoint_GetMaxMotorForce(jointId)
}

// Get the prismatic joint current motor force, typically in newtons
fn C.b2PrismaticJoint_GetMotorForce(jointId JointId) f32
@[inline]
pub fn prismatic_joint_get_motor_force(jointId JointId) f32 {
	return C.b2PrismaticJoint_GetMotorForce(jointId)
}

/** @} */

/**
 * @defgroup revolute_joint Revolute Joint
 * @brief A revolute joint allows for relative rotation in the 2D plane with no relative translation.
 * 
 * The revolute joint is probably the most common joint. It can be used for ragdolls and chains.
 * Also called a *hinge* or *pin* joint.
 * @{
 */

// Create a revolute joint
//	@see b2RevoluteJointDef for details
fn C.b2CreateRevoluteJoint(worldId WorldId, def &RevoluteJointDef) JointId
@[inline]
pub fn create_revolute_joint(worldId WorldId, def &RevoluteJointDef) JointId {
	return C.b2CreateRevoluteJoint(worldId, def)
}

// Enable/disable the revolute joint spring
fn C.b2RevoluteJoint_EnableSpring(jointId JointId, enableSpring bool)
@[inline]
pub fn revolute_joint_enable_spring(jointId JointId, enableSpring bool) {
	C.b2RevoluteJoint_EnableSpring(jointId, enableSpring)
}

// Set the revolute joint spring stiffness in Hertz
fn C.b2RevoluteJoint_SetSpringHertz(jointId JointId, hertz f32)
@[inline]
pub fn revolute_joint_set_spring_hertz(jointId JointId, hertz f32) {
	C.b2RevoluteJoint_SetSpringHertz(jointId, hertz)
}

// Get the revolute joint spring stiffness in Hertz
fn C.b2RevoluteJoint_GetSpringHertz(jointId JointId) f32
@[inline]
pub fn revolute_joint_get_spring_hertz(jointId JointId) f32 {
	return C.b2RevoluteJoint_GetSpringHertz(jointId) 
}

// Set the revolute joint spring damping ratio, non-dimensional
fn C.b2RevoluteJoint_SetSpringDampingRatio(jointId JointId, dampingRatio f32)
@[inline]
pub fn revolute_joint_set_spring_damping_ratio(jointId JointId, dampingRatio f32){
	C.b2RevoluteJoint_SetSpringDampingRatio(jointId, dampingRatio)
}

// Get the revolute joint spring damping ratio, non-dimensional
fn C.b2RevoluteJoint_GetSpringDampingRatio(jointId JointId) f32
@[inline]
pub fn revolute_joint_get_spring_damping_ratio(jointId JointId) f32 {
	return C.b2RevoluteJoint_GetSpringDampingRatio(jointId)
}

// Get the revolute joint current angle in radians relative to the reference angle
//	@see b2RevoluteJointDef::referenceAngle
fn C.b2RevoluteJoint_GetAngle(jointId JointId) f32
@[inline]
pub fn revolute_joint_get_angle(jointId JointId) f32 {
	return C.b2RevoluteJoint_GetAngle(jointId)
}

// Enable/disable the revolute joint limit
fn C.b2RevoluteJoint_EnableLimit(jointId JointId, enableLimit bool)
@[inline]
pub fn revolute_joint_enable_limit(jointId JointId, enableLimit bool) {
	C.b2RevoluteJoint_EnableLimit(jointId, enableLimit)
}

// Is the revolute joint limit enabled?
fn C.b2RevoluteJoint_IsLimitEnabled(jointId JointId) bool
@[inline]
pub fn revolute_joint_is_limit_enabled(jointId JointId) bool {
	return C.b2RevoluteJoint_IsLimitEnabled(jointId)
}

// Get the revolute joint lower limit in radians
fn C.b2RevoluteJoint_GetLowerLimit(jointId JointId) f32
@[inline]
pub fn revolute_joint_get_lower_limit(jointId JointId) f32  {
	return C.b2RevoluteJoint_GetLowerLimit(jointId)
}

// Get the revolute joint upper limit in radians
fn C.b2RevoluteJoint_GetUpperLimit(jointId JointId) f32
@[inline]
pub fn revolute_joint_get_upper_limit(jointId JointId) f32 {
	return C.b2RevoluteJoint_GetUpperLimit(jointId)
}

// Set the revolute joint limits in radians
fn C.b2RevoluteJoint_SetLimits(jointId JointId, lower f32, upper f32)
@[inline]
pub fn revolute_joint_set_limits(jointId JointId, lower f32, upper f32) {
	C.b2RevoluteJoint_SetLimits(jointId, lower, upper)
}

// Enable/disable a revolute joint motor
fn C.b2RevoluteJoint_EnableMotor(jointId JointId, enableMotor bool)
@[inline]
pub fn revolute_joint_enable_motor(jointId JointId, enableMotor bool) {
	C.b2RevoluteJoint_EnableMotor(jointId, enableMotor)
}

// Is the revolute joint motor enabled?
fn C.b2RevoluteJoint_IsMotorEnabled(jointId JointId) bool
@[inline]
pub fn revolute_joint_is_motor_enabled(jointId JointId) bool {
	return C.b2RevoluteJoint_IsMotorEnabled(jointId)
}

// Set the revolute joint motor speed in radians per second
fn C.b2RevoluteJoint_SetMotorSpeed(jointId JointId, motorSpeed f32)
@[inline]
pub fn revolute_joint_set_motor_speed(jointId JointId, motorSpeed f32) {
	C.b2RevoluteJoint_SetMotorSpeed(jointId, motorSpeed)
}

// Get the revolute joint motor speed in radians per second
fn C.b2RevoluteJoint_GetMotorSpeed(jointId JointId) f32
@[inline]
pub fn revolute_joint_get_motor_speed(jointId JointId) f32 {
	return C.b2RevoluteJoint_GetMotorSpeed(jointId) 
}

// Get the revolute joint current motor torque, typically in newton-meters
fn C.b2RevoluteJoint_GetMotorTorque(jointId JointId) f32
@[inline]
pub fn revolute_joint_get_motor_torque(jointId JointId) f32 {
	return C.b2RevoluteJoint_GetMotorTorque(jointId) 
}

// Set the revolute joint maximum motor torque, typically in newton-meters
fn C.b2RevoluteJoint_SetMaxMotorTorque(jointId JointId, torque f32)
@[inline]
pub fn revolute_joint_set_max_motor_torque(jointId JointId, torque f32) {
	C.b2RevoluteJoint_SetMaxMotorTorque(jointId, torque)
}

// Get the revolute joint maximum motor torque, typically in newton-meters
fn C.b2RevoluteJoint_GetMaxMotorTorque(jointId JointId) f32
@[inline]
pub fn revolute_joint_get_max_motor_torque(jointId JointId) f32 {
	return C.b2RevoluteJoint_GetMaxMotorTorque(jointId)
}

/**@}*/

/**
 * @defgroup weld_joint Weld Joint
 * @brief A weld joint fully constrains the relative transform between two bodies while allowing for springiness
 * 
 * A weld joint constrains the relative rotation and translation between two bodies. Both rotation and translation
 * can have damped springs.
 *
 * @note The accuracy of weld joint is limited by the accuracy of the solver. Long chains of weld joints may flex.
 * @{
 */

// Create a weld joint
//	@see b2WeldJointDef for details
fn C.b2CreateWeldJoint(worldId WorldId, def &WeldJointDef) JointId
@[inline]
pub fn create_weld_joint(worldId WorldId, def &WeldJointDef) JointId {
	return C.b2CreateWeldJoint(worldId, def)
}

// Set the weld joint linear stiffness in Hertz. 0 is rigid.
fn C.b2WeldJoint_SetLinearHertz(jointId JointId, hertz f32)
@[inline]
pub fn weld_joint_set_linear_hertz(jointId JointId, hertz f32) {
	C.b2WeldJoint_SetLinearHertz(jointId, hertz)
}

// Get the weld joint linear stiffness in Hertz
fn C.b2WeldJoint_GetLinearHertz(jointId JointId) f32
@[inline]
pub fn weld_joint_get_linear_hertz(jointId JointId) f32 {
	return C.b2WeldJoint_GetLinearHertz(jointId)
}

// Set the weld joint linear damping ratio (non-dimensional)
fn C.b2WeldJoint_SetLinearDampingRatio(jointId JointId, dampingRatio f32)
@[inline]
pub fn weld_joint_set_linear_damping_ratio(jointId JointId, dampingRatio f32) {
	C.b2WeldJoint_SetLinearDampingRatio(jointId, dampingRatio)
}

// Get the weld joint linear damping ratio (non-dimensional)
fn C.b2WeldJoint_GetLinearDampingRatio(jointId JointId) f32
@[inline]
pub fn weld_joint_get_linear_damping_ratio(jointId JointId) f32 {
	return C.b2WeldJoint_GetLinearDampingRatio(jointId)
}

// Set the weld joint angular stiffness in Hertz. 0 is rigid.
fn C.b2WeldJoint_SetAngularHertz(jointId JointId, hertz f32)
@[inline]
pub fn weld_joint_set_angular_hertz(jointId JointId, hertz f32) {
	C.b2WeldJoint_SetAngularHertz(jointId, hertz)
}

// Get the weld joint angular stiffness in Hertz
fn C.b2WeldJoint_GetAngularHertz(jointId JointId) f32
@[inline]
pub fn weld_joint_get_angular_hertz(jointId JointId) f32 {
	return C.b2WeldJoint_GetAngularHertz(jointId)
}

// Set weld joint angular damping ratio, non-dimensional
fn C.b2WeldJoint_SetAngularDampingRatio(jointId JointId, dampingRatio f32)
@[inline]
pub fn weld_joint_set_angular_damping_ratio(jointId JointId, dampingRatio f32) {
	C.b2WeldJoint_SetAngularDampingRatio(jointId, dampingRatio)
}

// Get the weld joint angular damping ratio, non-dimensional
fn C.b2WeldJoint_GetAngularDampingRatio(jointId JointId) f32
@[inline]
pub fn weld_joint_get_angular_damping_ratio(jointId JointId) f32 {
	return C.b2WeldJoint_GetAngularDampingRatio(jointId)
}

/** @} */

/**
 * @defgroup wheel_joint Wheel Joint
 * The wheel joint can be used to simulate wheels on vehicles.
 *
 * The wheel joint restricts body B to move along a local axis in body A. Body B is free to
 * rotate. Supports a linear spring, linear limits, and a rotational motor.
 *
 * @{
 */

// Create a wheel joint
//	@see b2WheelJointDef for details
fn C.b2CreateWheelJoint(worldId WorldId, def &WheelJointDef) JointId
@[inline]
pub fn create_wheel_joint(worldId WorldId, def &WheelJointDef) JointId {
	return C.b2CreateWheelJoint(worldId, def)
}

// Enable/disable the wheel joint spring
fn C.b2WheelJoint_EnableSpring(jointId JointId, enableSpring bool)
@[inline]
pub fn wheel_joint_enable_spring(jointId JointId, enableSpring bool) {
	C.b2WheelJoint_EnableSpring(jointId, enableSpring)
}

// Is the wheel joint spring enabled?
fn C.b2WheelJoint_IsSpringEnabled(jointId JointId) bool
@[inline]
pub fn wheel_joint_is_spring_enabled(jointId JointId) bool {
	return C.b2WheelJoint_IsSpringEnabled(jointId)
}

// Set the wheel joint stiffness in Hertz
fn C.b2WheelJoint_SetSpringHertz(jointId JointId, hertz f32)
@[inline]
pub fn wheel_joint_set_spring_hertz(jointId JointId, hertz f32) {
	C.b2WheelJoint_SetSpringHertz(jointId, hertz)
}

// Get the wheel joint stiffness in Hertz
fn C.b2WheelJoint_GetSpringHertz(jointId JointId) f32
@[inline]
pub fn wheel_joint_get_spring_hertz(jointId JointId) f32 {
	return C.b2WheelJoint_GetSpringHertz(jointId)
}

// Set the wheel joint damping ratio, non-dimensional
fn C.b2WheelJoint_SetSpringDampingRatio(jointId JointId, dampingRatio f32)
@[inline]
pub fn wheel_joint_set_spring_damping_ratio(jointId JointId, dampingRatio f32) {
	C.b2WheelJoint_SetSpringDampingRatio(jointId, dampingRatio)
}

// Get the wheel joint damping ratio, non-dimensional
fn C.b2WheelJoint_GetSpringDampingRatio(jointId JointId) f32
@[inline]
pub fn wheel_joint_get_spring_damping_ratio(jointId JointId) f32 {
	return C.b2WheelJoint_GetSpringDampingRatio(jointId)
}

// Enable/disable the wheel joint limit
fn C.b2WheelJoint_EnableLimit(jointId JointId, enableLimit bool)
@[inline]
pub fn wheel_joint_enable_limit(jointId JointId, enableLimit bool) {
	C.b2WheelJoint_EnableLimit(jointId, enableLimit)
}

// Is the wheel joint limit enabled?
fn C.b2WheelJoint_IsLimitEnabled(jointId JointId) bool
@[inline]
pub fn wheel_joint_is_limit_enabled(jointId JointId) bool {
	return C.b2WheelJoint_IsLimitEnabled(jointId)
}

// Get the wheel joint lower limit
fn C.b2WheelJoint_GetLowerLimit(jointId JointId) f32
@[inline]
pub fn wheel_joint_get_lower_limit(jointId JointId) f32 {
	return C.b2WheelJoint_GetLowerLimit(jointId)
}

// Get the wheel joint upper limit
fn C.b2WheelJoint_GetUpperLimit(jointId JointId) f32
@[inline]
pub fn wheel_joint_get_upper_limit(jointId JointId) f32 {
	return C.b2WheelJoint_GetUpperLimit(jointId)
}

// Set the wheel joint limits
fn C.b2WheelJoint_SetLimits(jointId JointId, lower f32, upper f32)
@[inline]
pub fn wheel_joint_set_limits(jointId JointId, lower f32, upper f32) {
	C.b2WheelJoint_SetLimits(jointId, lower, upper)
}

// Enable/disable the wheel joint motor
fn C.b2WheelJoint_EnableMotor(jointId JointId, enableMotor bool)
@[inline]
pub fn wheel_joint_enable_motor(jointId JointId, enableMotor bool) {
	C.b2WheelJoint_EnableMotor(jointId, enableMotor)
}

// Is the wheel joint motor enabled?
fn C.b2WheelJoint_IsMotorEnabled(jointId JointId) bool
@[inline]
pub fn wheel_joint_is_motor_enabled(jointId JointId) bool {
	return C.b2WheelJoint_IsMotorEnabled(jointId)
}

// Set the wheel joint motor speed in radians per second
fn C.b2WheelJoint_SetMotorSpeed(jointId JointId, motorSpeed f32)
@[inline]
pub fn wheel_joint_set_motor_speed(jointId JointId, motorSpeed f32) {
	C.b2WheelJoint_SetMotorSpeed(jointId, motorSpeed)
}

// Get the wheel joint motor speed in radians per second
fn C.b2WheelJoint_GetMotorSpeed(jointId JointId) f32
@[inline]
pub fn wheel_joint_get_motor_speed(jointId JointId) f32 {
	return C.b2WheelJoint_GetMotorSpeed(jointId)
}

// Set the wheel joint maximum motor torque, typically in newton-meters
fn C.b2WheelJoint_SetMaxMotorTorque(jointId JointId, torque f32)
@[inline]
pub fn wheel_joint_set_max_motor_torque(jointId JointId, torque f32) {
	C.b2WheelJoint_SetMaxMotorTorque(jointId, torque)
}

// Get the wheel joint maximum motor torque, typically in newton-meters
fn C.b2WheelJoint_GetMaxMotorTorque(jointId JointId) f32
@[inline]
pub fn wheel_joint_get_max_motor_torque(jointId JointId) f32 {
	return C.b2WheelJoint_GetMaxMotorTorque(jointId) 
}

// Get the wheel joint current motor torque, typically in newton-meters
fn C.b2WheelJoint_GetMotorTorque(jointId JointId) f32
@[inline]
pub fn wheel_joint_get_motor_torque(jointId JointId) f32 {
	return C.b2WheelJoint_GetMotorTorque(jointId)
}

/**@}*/

/**@}*/

// end box2d.h
