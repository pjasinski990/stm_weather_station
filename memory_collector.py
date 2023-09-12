import os
import subprocess
import argparse

def analyze_nm_output(object_file):
    memory_by_section = {'text': 0, 'bss': 0, 'data': 0}
    try:
        command_output = subprocess.run(['arm-none-eabi-nm', '--print-size', object_file], capture_output=True, text=True, check=True)
        lines = command_output.stdout.split('\n')
    except subprocess.CalledProcessError:
        return "An error occurred while running the nm command."
    
    for line in lines:
        tokens = line.split()
        if len(tokens) < 3:
            continue
        
        try:
            size = int(tokens[1], 16)
        except ValueError:
            continue
        
        type_char = tokens[2].upper()
        if type_char == 'T':
            memory_by_section['text'] += size
        elif type_char == 'B':
            memory_by_section['bss'] += size
        elif type_char == 'D':
            memory_by_section['data'] += size

    return memory_by_section

def find_c_files(src_dir):
    for root, _, files in os.walk(src_dir):
        for file in files:
            if file.endswith('.c'):
                yield os.path.join(root, file)

def find_corresponding_o_files(c_files, build_dir):
    for c_file in c_files:
        basename = os.path.basename(c_file)[:-2]  # remove .c extension
        for root, _, files in os.walk(build_dir):
            for file in files:
                if file == f"{basename}.o":
                    yield os.path.join(root, file)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyze memory usage of C source files in a build.')
    parser.add_argument('src_dir', type=str, help='Directory containing the source (.c) files')
    parser.add_argument('build_dir', type=str, help='Directory containing the build (.o) files')
    args = parser.parse_args()

    c_files = list(find_c_files(args.src_dir))
    o_files = find_corresponding_o_files(c_files, args.build_dir)

    sums = {'text': 0, 'bss': 0, 'data': 0}
    for o_file in o_files:
        memory_usage = analyze_nm_output(o_file)
        for key, value in memory_usage.items():
            sums[key] += value
        print(f'Memory usage for {o_file}: {memory_usage}')
    print(f'Sums: {sums}')
    print(f'Total: {sum(sums.values())}')
