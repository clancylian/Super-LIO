
import re

# 读取 ROSWrapper.cpp
with open('/workspace/src/super_lio/src/ros/ROSWrapper.cpp', 'r') as f:
    content = f.read()

# 定义一个函数来替换采样循环
def replace_sampling_loop(match):
    loop_var = match.group(1)
    start_idx = match.group(2)
    total_size = match.group(3)
    loop_body = match.group(4)
    
    new_code = f'''
    {{
        std::size_t {loop_var} = g_filter_offset;
        std::size_t last_idx = 0;
        for(; {loop_var} < {total_size}; {loop_var} += g_filter_rate){{
            last_idx = {loop_var};
            {loop_body}
        }}
        // 更新偏移量，用于下一次采样
        if(g_filter_rate > 0) {{
            g_filter_offset = (g_filter_offset + 1) % g_filter_rate;
        }}
    }}
    '''
    return new_code

# 首先处理 livoxHandler 中的循环
pattern1 = r'for\(std::size_t _i = 0; _i < ptsize; _i \+= g_filter_rate\)\{([^}]+)\}'
content = re.sub(pattern1, replace_sampling_loop, content)

# 处理 HESAI16 中的循环
pattern2 = r'for\(std::size_t i = 0; i < pl_orig\.size\(\); i \+= g_filter_rate\)\{([^}]+)\}'
content = re.sub(pattern2, replace_sampling_loop, content)

# 处理其他类似的循环（使用 int 而不是 std::size_t）
pattern3 = r'for\(int i = 0; i < plsize; i \+= g_filter_rate\)\{([^}]+)\}'
def replace_int_loop(match):
    loop_body = match.group(1)
    new_code = f'''
    {{
        int i = g_filter_offset;
        int last_idx = 0;
        for(; i < plsize; i += g_filter_rate){{
            last_idx = i;
            {loop_body}
        }}
        // 更新偏移量，用于下一次采样
        if(g_filter_rate > 0) {{
            g_filter_offset = (g_filter_offset + 1) % g_filter_rate;
        }}
    }}
    '''
    return new_code
content = re.sub(pattern3, replace_int_loop, content)

# 写回 ROSWrapper.cpp
with open('/workspace/src/super_lio/src/ros/ROSWrapper.cpp', 'w') as f:
    f.write(content)

# 现在修改 super_lio.cpp
with open('/workspace/src/super_lio/src/lio/super_lio.cpp', 'r') as f:
    sl_content = f.read()

# 找到并替换 super_lio.cpp 中的循环
sl_pattern = r'for\(std::size_t i = 0; i < ptsize; i \+= g_filter_rate\)\{([^}]+)\}'
def replace_sl_loop(match):
    loop_body = match.group(1)
    new_code = f'''
    {{
        std::size_t i = g_filter_offset;
        std::size_t last_idx = 0;
        for(; i < ptsize; i += g_filter_rate){{
            last_idx = i;
            {loop_body}
        }}
        // 更新偏移量，用于下一次采样
        if(g_filter_rate > 0) {{
            g_filter_offset = (g_filter_offset + 1) % g_filter_rate;
        }}
    }}
    '''
    return new_code

sl_content = re.sub(sl_pattern, replace_sl_loop, sl_content)

with open('/workspace/src/super_lio/src/lio/super_lio.cpp', 'w') as f:
    f.write(sl_content)

print("Modifications complete!")
