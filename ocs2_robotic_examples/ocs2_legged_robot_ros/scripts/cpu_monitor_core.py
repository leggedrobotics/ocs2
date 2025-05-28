import psutil
import time
import signal
import sys

output_file = f"cpu_frequencies_{time.strftime('%Y-%m-%d-%H-%M-%S')}.log"
max_freqs = []
total_freqs = []
sample_counts = []

def record_cpu_frequencies():
    global max_freqs, total_freqs, sample_counts
    with open(output_file, "a") as f:
        while True:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            # Get frequency for each CPU core
            cpu_freqs = psutil.cpu_freq(percpu=True)
            # init frequency lists
            if not max_freqs:
                max_freqs = [0.0] * len(cpu_freqs)
                total_freqs = [0.0] * len(cpu_freqs)
                sample_counts = [0] * len(cpu_freqs)
            # record frequency and update max frequency and total frequency
            freq_data = [f"{timestamp}"]
            for i, freq in enumerate(cpu_freqs):
                current_freq = freq.current
                freq_data.append(f"Core {i}: {current_freq} MHz")
                if current_freq > max_freqs[i]:
                    max_freqs[i] = current_freq
                total_freqs[i] += current_freq
                sample_counts[i] += 1
            f.write(", ".join(freq_data) + "\n")
            f.flush()
            time.sleep(1)

def signal_handler(sig, frame):
    print("\nCtrl+C detected. Printing highest and average frequencies recorded for each core:")
    for i, max_freq in enumerate(max_freqs):
        avg_freq = total_freqs[i] / sample_counts[i] if sample_counts[i] > 0 else 0
        print(f"Core {i}: max {round(max_freq)} MHz, avg {round(avg_freq)} MHz")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    print(f"Recording CPU frequencies to {output_file}. Press Ctrl+C to stop.")
    record_cpu_frequencies()

