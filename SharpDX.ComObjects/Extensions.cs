using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpDX.ComObjects
{
    public static class Extensions
    {
        public static int ReadInt(this DataStream ds)
        {
            return ds.Read<int>();
        }
    }
}
