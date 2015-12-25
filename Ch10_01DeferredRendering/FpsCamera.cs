using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpDX;
using SharpDX.Mathematics.Interop;

namespace Ch10_01DeferredRendering
{
    public static partial class MathF
    {
        public static readonly float Sqrt2 = (float)Math.Sqrt(2);


        private static readonly Random _rand = new Random();
        public const float PI = (float)Math.PI;

        public static float Sin(float a)
        {
            return (float)Math.Sin(a);
        }

        public static float Cos(float a)
        {
            return (float)Math.Cos(a);
        }

        public static float ToRadians(float degrees)
        {
            return PI * degrees / 180.0f;
        }

        public static float ToDegrees(float radians)
        {
            return radians * (180.0f / PI);
        }

        public static float Clamp(float value, float min, float max)
        {
            return Math.Max(min, Math.Min(value, max));
        }
        public static float Clamp<T>(float value, float min, float max)
        {
            return Math.Max(min, Math.Min(value, max));
        }

        public static int Rand()
        {
            return _rand.Next();
        }

        public static float Rand(float min, float max)
        {
            return min + (float)_rand.NextDouble() * (max - min);
        }

        public static Matrix InverseTranspose(Matrix m)
        {

            var a = m;
            a.M41 = a.M42 = a.M43 = 0;
            a.M44 = 1;

            return Matrix.Transpose(Matrix.Invert(a));
        }

        public static float Tan(float a)
        {
            return (float)Math.Tan(a);
        }

        public static float Atan(float f)
        {
            return (float)Math.Atan(f);
        }


        // heightmap functions
        public static float Noise(int x)
        {
            x = (x << 13) ^ x;
            return (1.0f - ((x * (x * x * 15731) + 1376312589) & 0x7fffffff) / 1073741824.0f);
        }

        public static float CosInterpolate(float v1, float v2, float a)
        {
            var angle = a * PI;
            var prc = (1.0f - LookupCos(angle)) * 0.5f;
            return v1 * (1.0f - prc) + v2 * prc;
        }
        public static float PerlinNoise2D(int seed, float persistence, int octave, float x, float y)
        {
            var freq = (float)Math.Pow(2.0f, octave);
            var amp = (float)Math.Pow(persistence, octave);
            var tx = x * freq;
            var ty = y * freq;
            var txi = (int)tx;
            var tyi = (int)ty;
            var fracX = tx - txi;
            var fracY = ty - tyi;

            var v1 = Noise(txi + tyi * 57 + seed);
            var v2 = Noise(txi + 1 + tyi * 57 + seed);
            var v3 = Noise(txi + (tyi + 1) * 57 + seed);
            var v4 = Noise(txi + 1 + (tyi + 1) * 57 + seed);

            var i1 = CosInterpolate(v1, v2, fracX);
            var i2 = CosInterpolate(v3, v4, fracX);
            var f = CosInterpolate(i1, i2, fracY) * amp;
            return f;
        }

        public static float Sqrt(float f)
        {
            return (float)Math.Sqrt(f);
        }

        public static float Pow(float x, float y)
        {
            return (float)Math.Pow(x, y);
        }

        public static Vector3 RandVector(Vector3 min, Vector3 max)
        {
            return new Vector3(Rand(min.X, max.X), Rand(min.Y, max.Y), Rand(min.Z, max.Z));
        }

        public static int Rand(int max) { return _rand.Next(max); }

        public static float AngleFromXY(float x, float y)
        {
            float theta;
            if (x >= 0.0f)
            {
                theta = Atan(y / x);
                if (theta < 0.0f)
                {
                    theta += 2 * PI;
                }
            }
            else {
                theta = Atan(y / x) + PI;
            }
            return theta;
        }

        public static float Acos(float f)
        {
            return (float)Math.Acos(f);
        }
    }

    public class Frustum
    {
        private readonly Plane[] _frustum;

        public const int Left = 0;
        public const int Right = 1;
        public const int Bottom = 2;
        public const int Top = 3;
        public const int Near = 4;
        public const int Far = 5;

        public Frustum(Matrix vp)
        {
            _frustum = new[] {
                //left
                new Plane(vp.M14 + vp.M11, vp.M24 + vp.M21, vp.M34 + vp.M31, vp.M44 + vp.M41),
                // right
                new Plane(vp.M14 - vp.M11, vp.M24 - vp.M21, vp.M34 - vp.M31, vp.M44 - vp.M41),
                // bottom
                new Plane(vp.M14 + vp.M12, vp.M24 + vp.M22, vp.M34 + vp.M32, vp.M44 + vp.M42),
                // top
                new Plane(vp.M14 - vp.M12, vp.M24 - vp.M22, vp.M34 - vp.M32, vp.M44 - vp.M42),
                //near
                new Plane(vp.M13, vp.M23, vp.M33, vp.M43),
                //far
                new Plane(vp.M14 - vp.M13, vp.M24 - vp.M23, vp.M34 - vp.M33, vp.M44 - vp.M43)
            };
            foreach (var plane in Planes)
            {
                plane.Normalize();
            }
        }



        public Plane[] Planes { get { return _frustum; } }

        public static Frustum FromViewProj(Matrix vp)
        {
            var ret = new Frustum(vp);
            return ret;
        }

        // Return values: 0 = no intersection, 
        //                1 = intersection, 
        //                2 = box is completely inside frustum
        public int Intersect(BoundingBox box)
        {
            var totalIn = 0;

            foreach (var plane in Planes)
            {
                var intersection = plane.Intersects(ref box);
                if (intersection == PlaneIntersectionType.Back) return 0;
                if (intersection == PlaneIntersectionType.Front)
                {
                    totalIn++;
                }
            }
            if (totalIn == 6)
            {
                return 2;
            }
            return 1;
        }
    }

    public abstract class CameraBase
    {
        protected Frustum _frustum;
        public Vector3 Position { get; set; }
        public Vector3 Right { get; protected set; }
        public Vector3 Up { get; protected set; }
        public Vector3 Look { get; protected set; }
        public float NearZ { get; protected set; }
        public float FarZ { get; protected set; }
        public float Aspect { get; protected set; }
        public float FovY { get; protected set; }
        public float FovX
        {
            get
            {
                var halfWidth = 0.5f * NearWindowWidth;
                return 2.0f * MathF.Atan(halfWidth / NearZ);
            }
        }
        public float NearWindowWidth { get { return Aspect * NearWindowHeight; } }
        public float NearWindowHeight { get; protected set; }
        public float FarWindowWidth { get { return Aspect * FarWindowHeight; } }
        public float FarWindowHeight { get; protected set; }
        public Matrix View { get; protected set; }
        public Matrix Proj { get; protected set; }
        public Matrix ViewProj { get { return View * Proj; } }
        public Plane[] FrustumPlanes { get { return _frustum.Planes; } }

        protected CameraBase()
        {
            Position = new Vector3();
            Right = new Vector3(1, 0, 0);
            Up = new Vector3(0, 1, 0);
            Look = new Vector3(0, 0, 1);

            View = Matrix.Identity;
            Proj = Matrix.Identity;
            //SetLens(0.25f * MathF.PI, 1.0f, 1.0f, 1000.0f);
        }

        public abstract void LookAt(Vector3 pos, Vector3 target, Vector3 up);
        public abstract void Strafe(float d);
        public abstract void Walk(float d);
        public abstract void Pitch(float angle);
        public abstract void Yaw(float angle);
        public abstract void Zoom(float dr);
        public abstract void UpdateViewMatrix();

        public bool Visible(BoundingBox box)
        {
            return _frustum.Intersect(box) > 0;
        }

        public virtual void SetLens(float fovY, float aspect, float zn, float zf)
        {
            FovY = fovY;
            Aspect = aspect;
            NearZ = zn;
            FarZ = zf;

            NearWindowHeight = 2.0f * NearZ * MathF.Tan(0.5f * FovY);
            FarWindowHeight = 2.0f * FarZ * MathF.Tan(0.5f * FovY);

            Proj = Matrix.PerspectiveFovLH(FovY, Aspect, NearZ, FarZ);
        }

        /// <summary>
        /// Return picking ray from camera through sp on screen, in world-space
        /// </summary>
        /// <param name="sp"></param>
        /// <param name="screenDims"></param>
        /// <returns></returns>
        public Ray GetPickingRay(Vector2 sp, Vector2 screenDims)
        {
            var p = Proj;
            // convert screen pixel to view space
            var vx = (2.0f * sp.X / screenDims.X - 1.0f) / p.M11;
            var vy = (-2.0f * sp.Y / screenDims.Y + 1.0f) / p.M22;

            var ray = new Ray(new Vector3(), new Vector3(vx, vy, 1.0f));
            var v = View;
            var invView = Matrix.Invert(v);


            var toWorld = invView;

            ray = new Ray(Vector3.TransformCoordinate(ray.Position, toWorld), Vector3.TransformNormal(ray.Direction, toWorld));

            ray.Direction.Normalize();
            return ray;
        }

        public Vector3[] GetFrustumCorners()
        {
            var hNear = 2 * MathF.Tan(FovY / 2) * NearZ;
            var wNear = hNear * Aspect;

            var hFar = 2 * MathF.Tan(FovY / 2) * FarZ;
            var wFar = hFar * Aspect;

            var cNear = Position + Look * NearZ;
            var cFar = Position + Look * FarZ;

            return new[] {
                //ntl
                cNear + (Up*hNear/2) - (Right*wNear/2),
                //ntr
                cNear + (Up*hNear/2) + (Right*wNear/2),
                //nbl
                cNear - (Up *hNear/2) - (Right*wNear/2),
                //nbr
                cNear - (Up *hNear/2) + (Right*wNear/2),
                //ftl
                cFar + (Up*hFar/2) - (Right*wFar/2),
                //ftr
                cFar + (Up*hFar/2) + (Right*wFar/2),
                //fbl
                cFar - (Up *hFar/2) - (Right*wFar/2),
                //fbr
                cFar - (Up *hFar/2) + (Right*wFar/2),
            };

        }
    }
    public class FpsCamera : CameraBase
    {

        public override void LookAt(Vector3 pos, Vector3 target, Vector3 up)
        {
            Position = pos;
            Look = Vector3.Normalize(target - pos);
            Right = Vector3.Normalize(Vector3.Cross(up, Look));
            Up = Vector3.Cross(Look, Right);
        }

        public override void Strafe(float d)
        {
            Position += Right * d;
        }

        public override void Walk(float d)
        {
            Position += Look * d;
        }

        public override void Pitch(float angle)
        {
            var r = Matrix.RotationAxis(Right, angle);
            Up = Vector3.TransformNormal(Up, r);
            Look = Vector3.TransformNormal(Look, r);
        }

        public override void Yaw(float angle)
        {
            var r = Matrix.RotationY(angle);
            Right = Vector3.TransformNormal(Right, r);
            Up = Vector3.TransformNormal(Up, r);
            Look = Vector3.TransformNormal(Look, r);
        }
        public override void Zoom(float dr)
        {
            var newFov = MathF.Clamp(FovY + dr, 0.1f, MathF.PI / 2);
            SetLens(newFov, Aspect, NearZ, FarZ);
        }

        public override void UpdateViewMatrix()
        {
            var r = Right;
            var u = Up;
            var l = Look;
            var p = Position;

            l = Vector3.Normalize(l);
            u = Vector3.Normalize(Vector3.Cross(l, r));

            r = Vector3.Cross(u, l);

            var x = -Vector3.Dot(p, r);
            var y = -Vector3.Dot(p, u);
            var z = -Vector3.Dot(p, l);

            Right = r;
            Up = u;
            Look = l;

            var v = new Matrix();
            v[0, 0] = Right.X;
            v[1, 0] = Right.Y;
            v[2, 0] = Right.Z;
            v[3, 0] = x;

            v[0, 1] = Up.X;
            v[1, 1] = Up.Y;
            v[2, 1] = Up.Z;
            v[3, 1] = y;

            v[0, 2] = Look.X;
            v[1, 2] = Look.Y;
            v[2, 2] = Look.Z;
            v[3, 2] = z;

            v[0, 3] = v[1, 3] = v[2, 3] = 0;
            v[3, 3] = 1;

            View = v;

            _frustum = Frustum.FromViewProj(ViewProj);
        }
    }
}
