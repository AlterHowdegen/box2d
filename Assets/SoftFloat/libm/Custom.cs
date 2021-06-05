
namespace SoftFloat
{
    public static partial class libm
    {

        public static sfloat Clamp01(sfloat v){
            if(v > sfloat.One){
                return sfloat.One;
            }

            if(v < sfloat.Zero){
                return sfloat.Zero;
            }

            return v;
        }

        public static sfloat Clamp(sfloat v, sfloat min, sfloat max){
            if(v > max){
                return max;
            }

            if(v < min){
                return min;
            }

            return v;
        }

        public static sfloat Deg2Rad = (sfloat)0.0174532924F;
        //
        // Summary:
        //     Radians-to-degrees conversion constant (Read Only).
        public static sfloat Rad2Deg = (sfloat)57.29578F;

        public static sfloat Sign(sfloat v){
            return v < sfloat.Zero ? -sfloat.One : sfloat.One;
        }
    }
}
