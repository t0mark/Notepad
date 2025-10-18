"""Jetson 환경에서 torch.distributed 접근 시 발생하는 오류를 완화하기 위한 더미 모듈."""
import sys
import types

torch_distributed = types.ModuleType("torch.distributed")
torch_distributed.rpc = None

sys.modules["torch.distributed"] = torch_distributed
sys.modules["torch.distributed.rpc"] = torch_distributed
