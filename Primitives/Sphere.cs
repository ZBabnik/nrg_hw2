using System;
using MathNet.Numerics.Integration;

namespace PathTracer
{
    class Sphere : Shape
    {
        public double Radius { get; set; }
        public bool Outside { get; set; }
        public Sphere(double radius, bool outside, Transform objectToWorld)
        {
            Radius = radius;
            Outside = outside;
            ObjectToWorld = objectToWorld;
        }

        public override (double?, SurfaceInteraction) Intersect(Ray ray)
        {

            Ray r = WorldToObject.Apply(ray);

            // TODO: Compute quadratic sphere coefficients

            // TODO: Initialize _double_ ray coordinate values
            (double dx, double dy, double dz) = (r.d.x, r.d.y, r.d.z);
            (double ox, double oy, double oz) = (r.o.x, r.o.y, r.o.z);
            double a = dx * dx + dy * dy + dz * dz;
            double b = 2 * (dx * ox + dy * oy + dz * oz);
            double c = ox * ox + oy * oy + oz * oz - this.Radius * this.Radius;

            // TODO: Solve quadratic equation for _t_ values
            (bool zzz, double t0, double t1) = Utils.Quadratic(a, b, c);
            if (!zzz) 
                return (null, null);

            // TODO: Check quadric shape _t0_ and _t1_ for nearest intersection
            //double tShapeHit;
            //if (t1 < Renderer.Epsilon)
            //    return (null, null);
            //if (t0 < Renderer.Epsilon)
            //    tShapeHit = t1;
            //else
            //{
            //    Ray temp = new Ray(r.o, r.d);
            //    double d1 = Vector3.Dot(temp.Point(t0), temp.o);
            //    double d2 = Vector3.Dot(temp.Point(t1), temp.o);
            //    if (d1 == d2)
            //        return (null, null);
            //    else if (d1 > d2)
            //    {
            //        if (Outside)
            //            tShapeHit = t0;
            //        else
            //            tShapeHit = t1;
            //    }
            //    else
            //    {
            //        if (Outside)
            //            tShapeHit = t1;
            //        else
            //            tShapeHit = t0;
            //    }
            //}

            //if (t1 - t0 <= Renderer.Epsilon)
            //    return (null, null);

            //Ray temp = new Ray(r.o, r.d);
            //double d0 = Vector3.Dot(temp.o, temp.Point(t0));
            //double d1 = Vector3.Dot(temp.o, temp.Point(t1));
            //double tShapeHit;

            //if (d0 >= d1)
            //    tShapeHit = t0;
            //else
            //    tShapeHit = t1;

            if (Outside)
            {
                Console.WriteLine("HERE");

                //if (t1 < Renderer.Epsilon)
                //    return (null, null);
                //double tShapeHit = t0;
                //if (tShapeHit < Renderer.Epsilon)
                //{
                //    tShapeHit = t1;
                //}

                double tShapeHit;
                Ray temp = new Ray(r.o, r.d);
                Vector3 p_t0 = temp.Point(t0);
                Vector3 p_t1 = temp.Point(t1);
                double d0 = Math.Sqrt(Math.Pow(temp.o.x - p_t0.x, 2) + Math.Pow(temp.o.y - p_t0.y, 2) + Math.Pow(temp.o.z - p_t0.z, 2));
                double d1 = Math.Sqrt(Math.Pow(temp.o.x - p_t1.x, 2) + Math.Pow(temp.o.y - p_t1.y, 2) + Math.Pow(temp.o.z - p_t1.z, 2));

                if (d0 > d1)
                {
                    tShapeHit = t1;
                }
                else {
                    tShapeHit = t0;
                }

                //// TODO: Compute sphere hit position and $\phi$
                Vector3 pHit = r.Point(tShapeHit);
                //pHit *= Radius / Math.Sqrt(pHit.x * pHit.x + pHit.y * pHit.y + pHit.z * pHit.z);

                //if (Math.Sqrt(pHit.x * pHit.x + pHit.y * pHit.y + pHit.z * pHit.z) >= this.Radius + Renderer.Epsilon)
                //    return (null, null);
                //if (Math.Sqrt(pHit.x * pHit.x + pHit.y * pHit.y + pHit.z * pHit.z) <= this.Radius - Renderer.Epsilon)
                //    return (null, null);

                //double phi = Math.Atan2(pHit.y, pHit.x);
                //if (phi < 0) 
                //    phi += 2 * Math.PI;
                //double theta = Math.Acos(Utils.Clamp(pHit.z / this.Radius, -1, 1));

                //double invR = 1 / Math.Sqrt(pHit.x * pHit.x + pHit.y * pHit.y);
                //Vector3 n = new Vector3(pHit.z * pHit.x * invR, pHit.z * pHit.y * invR, -this.Radius * Math.Sin(theta));

                Vector3 n = (ObjectToWorld.ApplyPoint(pHit) - ObjectToWorld.ApplyPoint(Vector3.ZeroVector)).Clone().Normalize();
                //Vector3 n = new Vector3(0, 0, 1);
                //Vector3 n = pHit.Clone() * this.Radius;

                // TODO: Return shape hit and surface interaction
                Vector3 dpdu = new Vector3(-pHit.y, pHit.x, 0);

                //Console.WriteLine("dpdu1 " + dpdu.x + " " + dpdu.y + " " + dpdu.z);

                n.Faceforward(dpdu);

                //Console.WriteLine("normal " + n.x + " " + n.y + " " + n.z);

                double abDot = Vector3.Dot(n, dpdu);
                double bbDot = Vector3.Dot(n, n);

                Vector3 proj = abDot / bbDot * n;

                dpdu -= proj;

                //Console.WriteLine("proj " + proj.x + " " + proj.y + " " + proj.z);

                //Console.WriteLine("dot " + Vector3.Dot(n, dpdu));

                var si = new SurfaceInteraction(pHit, n, -ray.d, dpdu, this);
                //var si = new SurfaceInteraction(pHit, new Vector3(0, 0, 1), -ray.d, dpdu, this);


                return (tShapeHit, ObjectToWorld.Apply(si));
            }
            else {

                //Console.WriteLine(r.o.x + " " + r.o.y + " " + r.o.z);
                //Console.WriteLine(t0);
                //Console.WriteLine(t1);
                //System.Environment.Exit(1);

                double tShapeHit;
                Ray temp = new Ray(r.o, r.d);
                Vector3 p_t0 = temp.Point(t0);
                Vector3 p_t1 = temp.Point(t1);

                Vector3 x = p_t0 - temp.o;
                Vector3 y = p_t1 - temp.o;

                double x_x = Vector3.Dot(x, temp.d);
                double y_x = Vector3.Dot(y, temp.d);

                //Console.WriteLine(temp.d.x + " " + temp.d.y + " " + temp.d.z);
                //Console.WriteLine(x_x);
                //Console.WriteLine(y_x);
                //System.Environment.Exit(1);

                if (x_x > 0)
                {
                    tShapeHit = t0;
                }
                else if (y_x > 0)
                {
                    tShapeHit = t1;
                }
                else {
                    return (null, null);
                }

                //// TODO: Compute sphere hit position and $\phi$
                Vector3 pHit = r.Point(tShapeHit);
                //pHit *= Radius / Math.Sqrt(pHit.x * pHit.x + pHit.y * pHit.y + pHit.z * pHit.z);

                //if (Math.Sqrt(pHit.x * pHit.x + pHit.y * pHit.y + pHit.z * pHit.z) >= this.Radius + Renderer.Epsilon)
                //    return (null, null);
                //if (Math.Sqrt(pHit.x * pHit.x + pHit.y * pHit.y + pHit.z * pHit.z) <= this.Radius - Renderer.Epsilon)
                //    return (null, null);

                //double phi = Math.Atan2(pHit.y, pHit.x);
                //if (phi < 0) 
                //    phi += 2 * Math.PI;
                //double theta = Math.Acos(Utils.Clamp(pHit.z / this.Radius, -1, 1));

                //double invR = 1 / Math.Sqrt(pHit.x * pHit.x + pHit.y * pHit.y);
                //Vector3 n = new Vector3(pHit.z * pHit.x * invR, pHit.z * pHit.y * invR, -this.Radius * Math.Sin(theta));

                Vector3 n = (ObjectToWorld.ApplyPoint(Vector3.ZeroVector) - ObjectToWorld.ApplyPoint(pHit)).Clone().Normalize();
                //Vector3 n = new Vector3(0, 0, 1);
                //Vector3 n = pHit.Clone() * this.Radius;

                // TODO: Return shape hit and surface interaction
                Vector3 dpdu = new Vector3(-pHit.y, pHit.x, 0);

                //Console.WriteLine("dpdu1 " + dpdu.x + " " + dpdu.y + " " + dpdu.z);

                dpdu.Faceforward(n);

                //Console.WriteLine("normal " + n.x + " " + n.y + " " + n.z);

                double abDot = Vector3.Dot(n, dpdu);
                double bbDot = Vector3.Dot(n, n);

                Vector3 proj = abDot / bbDot * n;

                dpdu -= proj;

                //Console.WriteLine("proj " + proj.x + " " + proj.y + " " + proj.z);

                //Console.WriteLine("dot " + Vector3.Dot(n, dpdu));

                var si = new SurfaceInteraction(pHit, n, -ray.d, dpdu, this);
                //var si = new SurfaceInteraction(pHit, new Vector3(0, 0, 1), -ray.d, dpdu, this);


                return (tShapeHit, ObjectToWorld.Apply(si));

            }

            // A dummy return example
            //double dummyHit = 0.0;
            //Vector3 dummyVector = new Vector3(0, 0, 0);
            //SurfaceInteraction dummySurfaceInteraction = new SurfaceInteraction(dummyVector, dummyVector, dummyVector, dummyVector, this);
            //return (dummyHit, dummySurfaceInteraction);
        }

        public override (SurfaceInteraction, double) Sample()
        {
            // TODO: Implement Sphere sampling
            if (Outside)
            {
                var pObj = this.Radius * Samplers.UniformSampleSphere();
                var n = new Vector3(pObj.x, pObj.y, pObj.z);
                var pdf = 1 / this.Area();
                var dpdu = new Vector3(-pObj.y, pObj.x, 0);

                SurfaceInteraction si = new SurfaceInteraction(pObj, n, Vector3.ZeroVector, dpdu, this);
                return (ObjectToWorld.Apply(si), pdf);
            }
            else {
                var pObj = this.Radius * Samplers.UniformSampleSphere();
                var n = new Vector3(pObj.x, pObj.y, pObj.z);
                var pdf = 1 / this.Area();
                var dpdu = new Vector3(-pObj.y, pObj.x, 0);

                SurfaceInteraction si = new SurfaceInteraction(pObj, -n, Vector3.ZeroVector, dpdu, this);
                return (ObjectToWorld.Apply(si), pdf);
            }

            // TODO: Return surface interaction and pdf

            // A dummy return example
            //double dummyPdf = 1.0;
            //Vector3 dummyVector = new Vector3(0, 0, 0);
            //SurfaceInteraction dummySurfaceInteraction = new SurfaceInteraction(dummyVector, dummyVector, dummyVector, dummyVector, this);
            //return (dummySurfaceInteraction, dummyPdf);
        }

        public override double Area() { return 4 * Math.PI * Radius * Radius; }

        public override double Pdf(SurfaceInteraction si, Vector3 wi)
        {
            var pCenter = this.WorldToObject.ApplyVector(Vector3.ZeroVector);
            var pOrigin = si.Point;

            if ((pOrigin - pCenter).LengthSquared() <= this.Radius * this.Radius)
                return (this as Shape).Pdf(si, wi);

            double sinThetaMax2 = Radius * Radius / (si.Point - pCenter).LengthSquared();
            double cosThetaMax = Math.Sqrt(Math.Max((double)0, 1 - sinThetaMax2));

            return 1 / (2 * Math.PI * (1 - cosThetaMax));

            //throw new NotImplementedException();
        }

    }
}
