from typing import List

class BaseException(Exception):
  def __init__(self, origin_stack:List[str], description:str):
    self._origin_stack = origin_stack
    self._description = description
  
  def add_in_stack(self, stack_update:List[str]):
    self._origin_stack = stack_update + self._origin_stack
  
  def describe(self):
    return {
      "origin" : '.'.join(self._origin_stack),
      "description": self._description
    }

  