import torch

def tensor_to_c_array(tensor, name):
    array = tensor.cpu().numpy()  # 确保在 CPU 上处理张量
    c_array = ""
    if len(array.shape) == 2:
        shape_str = f"[{array.shape[1]}][{array.shape[0]}]"
        c_array = f"static const float {name}{shape_str} = {{\n"
        for i in range(array.shape[1]):
            c_array += "    {" + ', '.join(map(str, array[:, i])) + "},\n"
        c_array += "};\n"
    elif len(array.shape) == 1:
        shape_str = f"[{array.shape[0]}]"
        c_array = f"static const float {name}{shape_str} = {{"
        c_array += ', '.join(map(str, array))
        c_array += "};\n"
    return c_array

def recursive_convert(name, param, c_arrays):
    if isinstance(param, torch.Tensor):  # 检查参数是否为张量
        c_name = name.replace('.', '_')
        c_arrays.append(tensor_to_c_array(param, c_name))
    elif isinstance(param, dict):  # 如果是字典，则递归处理
        for sub_name, sub_param in param.items():
            recursive_convert(f"{name}.{sub_name}", sub_param, c_arrays)

# 加载模型的状态字典
model_state_dict = torch.load('model_rarl.pt', map_location=torch.device('cpu'))

# 将每个参数转换为 C 数组
c_arrays = []
for name, param in model_state_dict.items():
    recursive_convert(name, param, c_arrays)

# 将所有 C 数组写入文件
with open('model_parameters_rarl.c', 'w') as f:
    for c_array in c_arrays:
        if c_array:  # 仅在 c_array 非空时写入文件
            f.write(c_array + '\n')
