import psutil
import time
import signal
import sys

output_file = f"cpu_usage_{time.strftime('%Y-%m-%d-%H-%M-%S')}.log"

cpu_usages = []
max_cpu_usage = 0.0

# process keyword options. Find the process id by these keywords
# 1~3 for legged_robot, 4~5 for franka and ridgeback
process_options = {
    1: "legged_robot_ddp_mpc",
    2: "legged_robot_sqp_mpc",
    3: "legged_robot_dummy",
    4: "mpc_node",
    5: "dummy_mrt_node"       
}

def find_process_by_keyword(keyword):
    """Find the process by keywords"""
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if keyword in proc.info['name'] or keyword in ' '.join(proc.info['cmdline']):
                return proc
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return None

def record_cpu_usage(proc):
    global max_cpu_usage
    with open(output_file, "a") as f:
        while True:
            try:
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                # Get cpu usage for each process
                cpu_usage = proc.cpu_percent(interval=1)
                # record cpu usage and update max cup usage
                cpu_usages.append(cpu_usage)
                if cpu_usage > max_cpu_usage:
                    max_cpu_usage = cpu_usage
                # write result to a file
                f.write(f"{timestamp}, CPU Usage: {cpu_usage}%\n")
                f.flush()
            except psutil.NoSuchProcess:
                print("\nProcess no longer exists.")
                calculate_and_print_cpu_usage()
                sys.exit(0)

def calculate_and_print_cpu_usage():
    if cpu_usages:
        avg_cpu_usage = round(sum(cpu_usages) / len(cpu_usages), 1)
        print(f"Average CPU Usage: {avg_cpu_usage}%")
        print(f"Maximum CPU Usage: {max_cpu_usage}%")
    else:
        print("No CPU usage data recorded.")

def signal_handler(sig, frame):
    print("\nCtrl+C detected.")
    calculate_and_print_cpu_usage()
    sys.exit(0)

if __name__ == "__main__":
    # if encounter Ctrl+C then save the results
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Select the process to monitor:")
    for key, value in process_options.items():
        print(f"{key} - {value}")
    
    # Get input
    try:
        option = int(input("Enter the number corresponding to the process: "))
        if option not in process_options:
            raise ValueError("Invalid option")
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    keyword = process_options[option]
    
    process = find_process_by_keyword(keyword)
    if process:
        print(f"Found process: {process.info['name']} (PID: {process.info['pid']})")
        print(f"Recording CPU usage to {output_file}. Press Ctrl+C to stop.")
        record_cpu_usage(process)
    else:
        print(f"No process found with keyword: {keyword}")
