/**
 * @file p2020.cs
 * @brief
 *
 * @author Johan Vanslembrouck
 */

using System;
using System.Threading;
using System.Threading.Tasks;

namespace p2020
{
    class p2020
    {
        public static void printcl(string text)
        {
            Console.WriteLine($"{Thread.CurrentThread.ManagedThreadId.ToString("D2")}: {text}");
        }

        /*
         * warning CS1998: This async method lacks 'await' operators and will run synchronously.
         * Consider using the 'await' operator to await non-blocking API calls, or 'await Task.Run(...)'
         * to do CPU-bound work on a background thread.
         */

        private static async Task<int> foo()
        {
            printcl("foo() await Task.Delay(1000);");
            await Task.Delay(1000);
            printcl("foo(): return 1;");
            return 1;
        }

        private static async Task<int> bar()
        {
            printcl("bar(): Task<int> f = foo();");
            Task<int> f = foo();
            printcl("bar(): int v = await f;");
            int v = await f;
            printcl($"bar(): return {v + 1};");
            return v + 1;
        }

        static void Main(string[] args)
        {
            printcl("Main(): Task b = bar();");
            Task<int> b = bar();
            printcl("Main(): b.Wait();");
            b.Wait();
            printcl("Main(): int v = b.Result;");
            int v = b.Result;
            printcl($"Main(): v = {v}");
            return;
        }
    }
}
