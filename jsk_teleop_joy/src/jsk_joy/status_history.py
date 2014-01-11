# status_history.py

class StatusHistory():
  def __init__(self, max_length=10):
    self.max_length = max_length
    self.buffer = []
  def add(self, status):
    self.buffer.append(status)
    if len(self.buffer) > self.max_length:
      self.buffer = self.buffer[1:self.max_length+1]
  def all(self, proc):
    for status in self.buffer:
      if not proc(status):
        return False
    return True
  def latest(self):
    return self.buffer[-1]
  def length(self):
    return len(self.buffer)
