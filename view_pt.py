import torch

def print_tensor_info(name, tensor):
    print(f"Parameter name: {name}")
    print(f"Shape: {tensor.shape}")
    print(f"Values: {tensor}\n")

def view_pt_file(file_path):
    # 加载模型的状态字典
    model_state_dict = torch.load(file_path, map_location=torch.device('cpu'))
    
    # 递归打印字典中的所有张量
    def recursive_print(name, item):
        if isinstance(item, torch.Tensor):
            print_tensor_info(name, item)
        elif isinstance(item, dict):
            for sub_name, sub_item in item.items():
                recursive_print(f"{name}.{sub_name}", sub_item)

    # 开始递归打印
    for name, param in model_state_dict.items():
        recursive_print(name, param)

# 替换为您的 .pt 文件路径
file_path = 'model_test.pt'

# 查看 .pt 文件内容
view_pt_file(file_path)
