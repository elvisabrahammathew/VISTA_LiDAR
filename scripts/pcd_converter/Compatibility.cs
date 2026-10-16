using System;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace PcdConverter
{
    // Small replacements for APIs not available on .NET Framework 4.7.2.
    public static class Compatibility
    {
        public static async Task<T> WaitAsync<T>(this Task<T> task, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            var cancelled = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            using (token.Register(() => cancelled.TrySetResult(true)))
            {
                if (await Task.WhenAny(task, cancelled.Task).ConfigureAwait(false) != task)
                    throw new OperationCanceledException(token);
                return await task.ConfigureAwait(false);
            }
        }

        public static async Task WaitAsync(this Task task, TimeSpan timeout)
        {
            using (var cancellation = new CancellationTokenSource(timeout))
            {
                var cancelled = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
                using (cancellation.Token.Register(() => cancelled.TrySetResult(true)))
                {
                    if (await Task.WhenAny(task, cancelled.Task).ConfigureAwait(false) != task)
                        throw new TimeoutException("The operation did not complete within the allowed time.");
                    await task.ConfigureAwait(false);
                }
            }
        }

        public static Task WaitForExitAsync(this Process process) => Task.Run(() => process.WaitForExit());

        // Windows argv quoting, including embedded quotes and trailing backslashes.
        // No shell is used, so &, %, ; and $ remain literal characters.
        public static string QuoteArguments(System.Collections.Generic.IEnumerable<string> arguments) =>
            string.Join(" ", arguments.Select(QuoteArgument));

        private static string QuoteArgument(string argument)
        {
            var result = new StringBuilder("\"");
            int slashes = 0;
            foreach (char character in argument)
            {
                if (character == '\\') { slashes++; continue; }
                if (character == '"') result.Append('\\', slashes * 2 + 1).Append('"');
                else result.Append('\\', slashes).Append(character);
                slashes = 0;
            }
            return result.Append('\\', slashes * 2).Append('"').ToString();
        }
    }
}
