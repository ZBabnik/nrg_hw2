using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathTracer
{
    public class OrenNayar : BxDF
    {
        private Spectrum kd;
        private double cov;

        public OrenNayar(Spectrum r, double roughness)
        {
            kd = r;
            this.cov = roughness * roughness;
        }

        public override Spectrum f(Vector3 wo, Vector3 wi)
        {
            if (!Utils.SameHemisphere(wo, wi))
                return Spectrum.ZeroSpectrum;

            double A = 1 - (this.cov) / (2 * (this.cov + 0.33));
            double B = 0.45 * (this.cov) / (this.cov + 0.09);

            double r_o = Math.Sqrt(wo.x * wo.x + wo.y * wo.y + wo.z * wo.z);
            double r_i = Math.Sqrt(wi.x * wi.x + wi.y * wi.y + wi.z * wi.z);

            //double alpha = Math.Max(Math.Acos(wo.z / r_o), Math.Acos(wi.z / r_i));
            //double beta = Math.Min(Math.Acos(wo.z / r_o), Math.Acos(wi.z / r_i));

            //double cosPhi_o = Utils.CosPhi(wo);
            //double cosPhi_i = Utils.CosPhi(wi);

            //double cosTheta_o = Utils.CosTheta(wo);
            //double cosTheta_i = Utils.CosTheta(wi);

            double alpha = Math.Max(Math.Acos(wo.z / r_o), Math.Acos(wi.z / r_i));
            double beta = Math.Min(Math.Acos(wo.z / r_o), Math.Acos(wi.z / r_i));

            double phi_o = Math.Atan(wo.y / wo.x);
            double phi_i = Math.Atan(wi.y / wo.x);

            return kd * Utils.PiInv * (A + B * Math.Max(0, Math.Cos(phi_i - phi_o)) * Math.Sin(alpha) * Math.Tan(beta));
        }

        public override (Spectrum, Vector3, double) Sample_f(Vector3 wo)
        {
            var wi = Samplers.CosineSampleHemisphere();
            if (wo.z < 0)
                wi.z *= -1;
            double pdf = Pdf(wo, wi);
            return (f(wo, wi), wi, pdf);
        }

        public override double Pdf(Vector3 wo, Vector3 wi)
        {
            if (!Utils.SameHemisphere(wo, wi))
                return 0;

            return Math.Abs(wi.z) * Utils.PiInv; // wi.z == cosTheta
        }
    }
}
