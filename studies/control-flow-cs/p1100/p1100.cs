/**
 * @file p1100.cs
 * @brief
 *
 * @author Johan Vanslembrouck
 */

using System;
using System.Threading;
using System.Threading.Tasks;

namespace p1100
{
    class p1100
    {
        public static void printcl(string text)
        {
            Console.WriteLine($"{Thread.CurrentThread.ManagedThreadId.ToString("D2")}: {text}");
        }

        private static async Task<int> completes_synchronously(int i)
        {
            printcl($"completes_synchronously({i}): return {i};");
            return i;
        }

        private static async Task<int> loop_synchronously(int counter)
        {
            printcl($"loop_synchronously({counter})");
            int v = 0;
            for (int i = 0; i < counter; ++i)
            {
                printcl($"loop_synchronously(): Task<int> cs = completes_synchronously({i});");
                Task<int> cs = completes_synchronously(i);
                printcl("loop_synchronously(): v += await cs;");
                v += await cs;
                printcl($"loop_synchronously(): v = {v}");
            }
            printcl($"loop_synchronously(): return {v};");
            return v;
        }

        static void Main(string[] args)
        {
            printcl("Main(): Task<int> ls = loop_synchronously(4);");
            Task<int> ls = loop_synchronously(4);
            printcl("Main(): ls.Wait();");
            ls.Wait();
            printcl("Main(): int v = b.Result;");
            int v = ls.Result; 
            printcl($"Main(): v = {v}");
            return;
        }
    }
}
