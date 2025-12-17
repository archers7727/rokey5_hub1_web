-- ========================================
-- Tasks Table Schema Update
-- Simplify date columns and add auto-updating updated_at
-- ========================================

-- 1. Drop old columns
ALTER TABLE tasks
DROP COLUMN IF EXISTS started_at,
DROP COLUMN IF EXISTS completed_at;

-- 2. Add updated_at column with auto-update trigger
ALTER TABLE tasks
ADD COLUMN IF NOT EXISTS updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW();

-- 3. Create function to auto-update updated_at
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = NOW();
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- 4. Create trigger for tasks table
DROP TRIGGER IF EXISTS update_tasks_updated_at ON tasks;
CREATE TRIGGER update_tasks_updated_at
  BEFORE UPDATE ON tasks
  FOR EACH ROW
  EXECUTE FUNCTION update_updated_at_column();

-- 5. Add comment for documentation
COMMENT ON COLUMN tasks.created_at IS 'Task start time - when task was created/started';
COMMENT ON COLUMN tasks.updated_at IS 'Task end time - automatically updated on any change, represents completion time for completed tasks';

-- 6. For existing completed tasks, set updated_at to created_at if null
UPDATE tasks
SET updated_at = created_at
WHERE updated_at IS NULL;
