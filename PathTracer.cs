using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static PathTracer.Samplers;

namespace PathTracer
{
    class PathTracer
    {
        public Spectrum Li(Ray r, Scene s)
        {
            /* BASIC Implement
            repeat
                r <- random ray from camera
                L <- 0, β <- 1, nbounces <- 0
                repeat
                   isect <- intersect r with scene
                   if isect == null // no hit
                      break
                   wo <- -r
                   if isect == light // light hit
                      L<- β*Le(wo) // add light emitted
                      break
                   wi <- random ray from isect
                   (f,pr) <- bsdf(isect,wi,wo)
                   β <- β*f*|cosθ|/pr
                   r <- wi
                   nbounces <- nbounces+1
                AddSampleToImage(L,r)
            */


            /* Russian Roulette Implement

            (f,pr) <- bsdf(isect,wi,wo)
            β <- β*f*|cosθ|/pr
            r <- wi
            if nbounces>3
            q <- 1 - max(β)
            if random() < q
            break
            β <- β/(1-q)

             */
            var L = Spectrum.ZeroSpectrum;
            var beta = Spectrum.Create((double)1);
            int nbounces = 0;
            double q;

            while(nbounces < 20) {
                (double? mint, SurfaceInteraction si) = s.Intersect(r);

                // If nothing hit end
                if (si == null) {
                    break;
                }

                // If Light hit include it and finish
                if (si.Obj is Light) {
                    if (nbounces == 0)
                    {
                        L = beta * si.Le(si.Wo);
                        //Console.WriteLine(beta.c);
                        //Console.WriteLine(L.c);
                    }
                    break;
                }

                // Sample light from SurfaceInteraction
                // TODO
                Spectrum Ld = Light.UniformSampleOneLight(si, s);

                L = L.AddTo(beta * Ld);
                
                // Make new reflected Ray
                //Ray wi = si.SpawnRay(UniformSampleSphere());
                
                // Get BSDF of hit object
                (Spectrum f, Vector3 wi, double pr, bool p) = (si.Obj as Shape).BSDF.Sample_f(si.Wo, si);

                // Update beta
                beta = beta * f * Utils.AbsCosTheta(wi) / pr;
                // Set reflected Ray as original Ray
                r = new Ray(si.Point, wi);

                if (nbounces > 3) {
                    q = 1 - beta.Max();
                    if (ThreadSafeRandom.NextDouble() < q) {
                        break;
                    }
                    beta /= (1 - q);
                }

                nbounces += 1;
            }
            Console.WriteLine(L.c);
            return L;
        }

    } 
}
