-- ========================================
-- Update tasks table to new schema
-- Remove started_at, completed_at
-- Add updated_at with initial values for existing data
-- ========================================

-- Step 1: Add updated_at column if not exists
ALTER TABLE tasks
ADD COLUMN IF NOT EXISTS updated_at TIMESTAMP WITH TIME ZONE;

-- Step 2: Set updated_at for all existing tasks
-- For completed tasks, set updated_at = created_at (will show 0 duration, but estimated_time will be used as fallback)
-- For non-completed tasks, set updated_at = NOW()
UPDATE tasks
SET updated_at = CASE
  WHEN status = 'completed' THEN created_at
  WHEN status = 'failed' THEN created_at
  ELSE NOW()
END
WHERE updated_at IS NULL;

-- Step 3: Drop old columns
ALTER TABLE tasks
DROP COLUMN IF EXISTS started_at;

ALTER TABLE tasks
DROP COLUMN IF EXISTS completed_at;

-- Step 4: Create auto-update function
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = NOW();
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Step 5: Create trigger
DROP TRIGGER IF EXISTS update_tasks_updated_at ON tasks;
CREATE TRIGGER update_tasks_updated_at
  BEFORE UPDATE ON tasks
  FOR EACH ROW
  EXECUTE FUNCTION update_updated_at_column();

-- Step 6: Add comments
COMMENT ON COLUMN tasks.created_at IS 'Task start time - when task was created';
COMMENT ON COLUMN tasks.updated_at IS 'Task end time - auto-updated on any change';

-- Step 7: Verify the changes
SELECT
  id,
  material_id,
  mode_id,
  status,
  created_at,
  updated_at,
  EXTRACT(EPOCH FROM (updated_at - created_at)) as duration_seconds
FROM tasks
LIMIT 5;
