use crate::math::{Ray, Vec3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    /// Diffuse response at the projector wavelength, nominally 0..1.
    pub diffuse_reflectance: f64,
    /// Retroreflective multiplier useful for printed fiducials/tape.
    pub retroreflective_gain: f64,
    /// RMS-like surface roughness in 0..1; currently reported for downstream models.
    pub roughness: f64,
}

impl Default for Material {
    fn default() -> Self {
        Self {
            diffuse_reflectance: 0.65,
            retroreflective_gain: 1.0,
            roughness: 0.5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sphere {
    pub center: Vec3,
    pub radius_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Triangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
    pub double_sided: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cylinder {
    pub start: Vec3,
    pub end: Vec3,
    pub radius_m: f64,
    pub capped: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Geometry {
    Sphere(Sphere),
    Aabb(Aabb),
    Triangle(Triangle),
    Cylinder(Cylinder),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Primitive {
    pub geometry: Geometry,
    pub material: Material,
    /// Application-defined part/fixture identifier.
    pub tag: u32,
}

impl Primitive {
    pub const fn new(geometry: Geometry, material: Material, tag: u32) -> Self {
        Self {
            geometry,
            material,
            tag,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub distance_m: f64,
    pub position: Vec3,
    /// Geometric outward/front normal.
    pub normal: Vec3,
    pub primitive_index: usize,
    pub tag: u32,
    pub material: Material,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Scene {
    pub primitives: Vec<Primitive>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshError {
    TriangleIndexOutOfBounds {
        triangle_index: usize,
        vertex_index: u32,
    },
}

impl Scene {
    pub fn new(primitives: Vec<Primitive>) -> Self {
        Self { primitives }
    }

    pub fn push(&mut self, primitive: Primitive) {
        self.primitives.push(primitive);
    }

    /// Add an indexed triangle soup exported from CAD without pulling a mesh
    /// dependency into the WebAssembly build. Validation happens before mutation.
    pub fn extend_triangle_mesh(
        &mut self,
        vertices: &[Vec3],
        triangles: &[[u32; 3]],
        material: Material,
        tag: u32,
        double_sided: bool,
    ) -> Result<(), MeshError> {
        for (triangle_index, triangle) in triangles.iter().enumerate() {
            for &vertex_index in triangle {
                if vertex_index as usize >= vertices.len() {
                    return Err(MeshError::TriangleIndexOutOfBounds {
                        triangle_index,
                        vertex_index,
                    });
                }
            }
        }
        self.primitives.reserve(triangles.len());
        for triangle in triangles {
            self.push(Primitive::new(
                Geometry::Triangle(Triangle {
                    a: vertices[triangle[0] as usize],
                    b: vertices[triangle[1] as usize],
                    c: vertices[triangle[2] as usize],
                    double_sided,
                }),
                material,
                tag,
            ));
        }
        Ok(())
    }

    /// Closest opaque surface on a ray in the inclusive distance interval.
    pub fn intersect(&self, ray: Ray, t_min_m: f64, t_max_m: f64) -> Option<Hit> {
        if t_min_m > t_max_m || t_max_m < 0.0 {
            return None;
        }
        let mut closest = t_max_m;
        let mut best = None;
        for (primitive_index, primitive) in self.primitives.iter().copied().enumerate() {
            if let Some((distance_m, normal)) =
                intersect_geometry(primitive.geometry, ray, t_min_m.max(0.0), closest)
            {
                closest = distance_m;
                best = Some(Hit {
                    distance_m,
                    position: ray.at(distance_m),
                    normal,
                    primitive_index,
                    tag: primitive.tag,
                    material: primitive.material,
                });
            }
        }
        best
    }

    /// True when another surface blocks the open segment from `from` to `to`.
    pub fn occluded(&self, from: Vec3, to: Vec3, epsilon_m: f64) -> bool {
        let delta = to - from;
        let distance = delta.norm();
        if distance <= 2.0 * epsilon_m.max(0.0) {
            return false;
        }
        let Some(ray) = Ray::new(from, delta) else {
            return false;
        };
        self.intersect(ray, epsilon_m.max(0.0), distance - epsilon_m.max(0.0))
            .is_some()
    }
}

fn intersect_geometry(geometry: Geometry, ray: Ray, t_min: f64, t_max: f64) -> Option<(f64, Vec3)> {
    match geometry {
        Geometry::Sphere(s) => intersect_sphere(s, ray, t_min, t_max),
        Geometry::Aabb(a) => intersect_aabb(a, ray, t_min, t_max),
        Geometry::Triangle(t) => intersect_triangle(t, ray, t_min, t_max),
        Geometry::Cylinder(c) => intersect_cylinder(c, ray, t_min, t_max),
    }
}

fn intersect_sphere(sphere: Sphere, ray: Ray, t_min: f64, t_max: f64) -> Option<(f64, Vec3)> {
    if sphere.radius_m <= 0.0 {
        return None;
    }
    let oc = ray.origin - sphere.center;
    let half_b = oc.dot(ray.direction);
    let c = oc.norm_squared() - sphere.radius_m * sphere.radius_m;
    let discriminant = half_b * half_b - c;
    if discriminant < 0.0 {
        return None;
    }
    let root = discriminant.sqrt();
    for distance in [-half_b - root, -half_b + root] {
        if (t_min..=t_max).contains(&distance) {
            let normal = (ray.at(distance) - sphere.center) / sphere.radius_m;
            return Some((distance, normal));
        }
    }
    None
}

fn intersect_aabb(aabb: Aabb, ray: Ray, t_min: f64, t_max: f64) -> Option<(f64, Vec3)> {
    if aabb.min.x > aabb.max.x || aabb.min.y > aabb.max.y || aabb.min.z > aabb.max.z {
        return None;
    }
    let origins = [ray.origin.x, ray.origin.y, ray.origin.z];
    let directions = [ray.direction.x, ray.direction.y, ray.direction.z];
    let mins = [aabb.min.x, aabb.min.y, aabb.min.z];
    let maxs = [aabb.max.x, aabb.max.y, aabb.max.z];
    let axes = [Vec3::X, Vec3::Y, Vec3::Z];
    let mut near = f64::NEG_INFINITY;
    let mut far = f64::INFINITY;
    let mut near_normal = Vec3::ZERO;
    let mut far_normal = Vec3::ZERO;
    for axis in 0..3 {
        if directions[axis].abs() < 1.0e-15 {
            if origins[axis] < mins[axis] || origins[axis] > maxs[axis] {
                return None;
            }
            continue;
        }
        let inv = 1.0 / directions[axis];
        let mut a = (mins[axis] - origins[axis]) * inv;
        let mut b = (maxs[axis] - origins[axis]) * inv;
        let mut normal_a = -axes[axis];
        let mut normal_b = axes[axis];
        if a > b {
            core::mem::swap(&mut a, &mut b);
            core::mem::swap(&mut normal_a, &mut normal_b);
        }
        if a > near {
            near = a;
            near_normal = normal_a;
        }
        if b < far {
            far = b;
            far_normal = normal_b;
        }
        if near > far {
            return None;
        }
    }
    if (t_min..=t_max).contains(&near) {
        Some((near, near_normal))
    } else if (t_min..=t_max).contains(&far) {
        Some((far, far_normal))
    } else {
        None
    }
}

fn intersect_triangle(triangle: Triangle, ray: Ray, t_min: f64, t_max: f64) -> Option<(f64, Vec3)> {
    let e1 = triangle.b - triangle.a;
    let e2 = triangle.c - triangle.a;
    let p = ray.direction.cross(e2);
    let determinant = e1.dot(p);
    if triangle.double_sided {
        if determinant.abs() < 1.0e-14 {
            return None;
        }
    } else if determinant < 1.0e-14 {
        return None;
    }
    let inv_det = 1.0 / determinant;
    let t = ray.origin - triangle.a;
    let u = t.dot(p) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = t.cross(e1);
    let v = ray.direction.dot(q) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let distance = e2.dot(q) * inv_det;
    if distance < t_min || distance > t_max {
        return None;
    }
    let mut normal = e1.cross(e2).normalized()?;
    if triangle.double_sided && normal.dot(ray.direction) > 0.0 {
        normal = -normal;
    }
    Some((distance, normal))
}

fn intersect_cylinder(cylinder: Cylinder, ray: Ray, t_min: f64, t_max: f64) -> Option<(f64, Vec3)> {
    if cylinder.radius_m <= 0.0 {
        return None;
    }
    let axis_delta = cylinder.end - cylinder.start;
    let length = axis_delta.norm();
    let axis = axis_delta.normalized()?;
    let offset = ray.origin - cylinder.start;
    let d_axis = ray.direction.dot(axis);
    let o_axis = offset.dot(axis);
    let d_perp = ray.direction - axis * d_axis;
    let o_perp = offset - axis * o_axis;
    let a = d_perp.norm_squared();
    let half_b = d_perp.dot(o_perp);
    let c = o_perp.norm_squared() - cylinder.radius_m * cylinder.radius_m;
    let mut candidates: [(f64, Vec3); 4] = [(f64::INFINITY, Vec3::ZERO); 4];
    let mut count = 0;
    if a > 1.0e-15 {
        let discriminant = half_b * half_b - a * c;
        if discriminant >= 0.0 {
            let root = discriminant.sqrt();
            for distance in [(-half_b - root) / a, (-half_b + root) / a] {
                let axial = o_axis + distance * d_axis;
                if (t_min..=t_max).contains(&distance) && (0.0..=length).contains(&axial) {
                    let position = ray.at(distance);
                    let spine = cylinder.start + axis * axial;
                    candidates[count] = (distance, (position - spine).normalized()?);
                    count += 1;
                }
            }
        }
    }
    if cylinder.capped && d_axis.abs() > 1.0e-15 {
        for (axial, normal) in [(0.0, -axis), (length, axis)] {
            let distance = (axial - o_axis) / d_axis;
            if (t_min..=t_max).contains(&distance) {
                let p = offset + ray.direction * distance - axis * axial;
                if p.norm_squared() <= cylinder.radius_m * cylinder.radius_m {
                    candidates[count] = (distance, normal);
                    count += 1;
                }
            }
        }
    }
    candidates[..count]
        .iter()
        .copied()
        .min_by(|a, b| a.0.total_cmp(&b.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closest_hit_controls_occlusion() {
        let material = Material::default();
        let scene = Scene::new(vec![
            Primitive::new(
                Geometry::Sphere(Sphere {
                    center: Vec3::new(0.0, 0.0, 2.0),
                    radius_m: 0.25,
                }),
                material,
                2,
            ),
            Primitive::new(
                Geometry::Sphere(Sphere {
                    center: Vec3::new(0.0, 0.0, 1.0),
                    radius_m: 0.1,
                }),
                material,
                1,
            ),
        ]);
        let ray = Ray::new(Vec3::ZERO, Vec3::Z).unwrap();
        assert_eq!(scene.intersect(ray, 0.0, 10.0).unwrap().tag, 1);
        assert!(scene.occluded(Vec3::ZERO, Vec3::new(0.0, 0.0, 2.0), 1.0e-5));
    }

    #[test]
    fn finite_cylinder_hits_side_and_cap() {
        let cylinder = Geometry::Cylinder(Cylinder {
            start: Vec3::new(0.0, 0.0, 1.0),
            end: Vec3::new(0.0, 0.0, 2.0),
            radius_m: 0.2,
            capped: true,
        });
        let side = intersect_geometry(
            cylinder,
            Ray::new(Vec3::new(1.0, 0.0, 1.5), -Vec3::X).unwrap(),
            0.0,
            10.0,
        )
        .unwrap();
        assert!((side.0 - 0.8).abs() < 1.0e-12);
        let cap = intersect_geometry(cylinder, Ray::new(Vec3::ZERO, Vec3::Z).unwrap(), 0.0, 10.0)
            .unwrap();
        assert!((cap.0 - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn cad_triangle_import_is_validated_before_mutation() {
        let mut scene = Scene::default();
        let vertices = [Vec3::ZERO, Vec3::X, Vec3::Y];
        let error = scene
            .extend_triangle_mesh(&vertices, &[[0, 1, 9]], Material::default(), 7, false)
            .unwrap_err();
        assert_eq!(
            error,
            MeshError::TriangleIndexOutOfBounds {
                triangle_index: 0,
                vertex_index: 9,
            }
        );
        assert!(scene.primitives.is_empty());

        scene
            .extend_triangle_mesh(&vertices, &[[0, 1, 2]], Material::default(), 7, false)
            .unwrap();
        assert_eq!(scene.primitives.len(), 1);
    }
}
