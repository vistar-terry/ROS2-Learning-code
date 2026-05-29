#!/usr/bin/env python3
"""
get_ast.py - 调试 AST 生成的脚本
"""

import sys
from pathlib import Path
from rosidl_parser.definition import IdlLocator
from rosidl_parser.parser import get_ast_from_idl_string

def debug_single_idl(idl_path):
    print(f"解析: {idl_path}")
    path = Path(idl_path)
    locator = IdlLocator(
        basepath = path.parent,
        relative_path = path.name
    )
    
    # 解析 AST
    idl_string = locator.get_absolute_path().read_text(encoding='utf-8')
    tree = get_ast_from_idl_string(idl_string)
    
    return tree

if __name__ == "__main__":
    if len(sys.argv) > 1:
        idl_path = sys.argv[1]
        ast = debug_single_idl(idl_path)
        print()
        print(ast)
        print()
    else:
        print("用法: python get_ast.py <idl文件路径>")

