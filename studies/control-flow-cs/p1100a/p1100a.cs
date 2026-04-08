/**
 * @file p1100a.cs
 * @brief
 *
 * @author Johan Vanslembrouck
 */

using System;
using System.Threading;
using System.Threading.Tasks;

namespace p1100a
{
    class p1100a
    {
        public static void printcl(string text)
        {
            Console.WriteLine($"{Thread.CurrentThread.ManagedThreadId.ToString("D2")}: {text}");
        }

        private static async Task<int> completes_synchronously(int i)
        {
            return i;
        }

        private static async Task<int> loop_synchronously(int counter)
        {
            int v = 0;
            for (int i = 0; i < counter; ++i)
            {
                Task<int> b = completes_synchronously(i);
                v += await b;
            }
            printcl($"loop_synchronously() return {v};");
            return v;
        }

        static void Main(string[] args)
        {
            printcl("Main(): Task<int> ls = loop_synchronously(10000);");
            Task<int> ls = loop_synchronously(10000);
            printcl("Main(): ls.Wait();");
            ls.Wait();
            printcl("Main(): int v = ls.Result;");
            int v = ls.Result;
            printcl($"Main(): v = {v}");
            return;
        }
    }
}
