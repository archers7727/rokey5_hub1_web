-- ========================================
-- Add ON UPDATE CASCADE to tasks foreign keys
-- Allows updating material_id and mode_id when referenced rows change
-- ========================================

-- 1. Drop existing foreign key constraints
ALTER TABLE tasks
DROP CONSTRAINT IF EXISTS tasks_material_id_fkey,
DROP CONSTRAINT IF EXISTS tasks_mode_id_fkey;

-- 2. Add new foreign key constraints with ON UPDATE CASCADE
ALTER TABLE tasks
ADD CONSTRAINT tasks_material_id_fkey
  FOREIGN KEY (material_id)
  REFERENCES materials(id)
  ON UPDATE CASCADE
  ON DELETE RESTRICT,
ADD CONSTRAINT tasks_mode_id_fkey
  FOREIGN KEY (mode_id)
  REFERENCES modes(id)
  ON UPDATE CASCADE
  ON DELETE RESTRICT;

-- 3. Verify constraints
SELECT
  conname AS constraint_name,
  conrelid::regclass AS table_name,
  confrelid::regclass AS referenced_table,
  confupdtype AS on_update,
  confdeltype AS on_delete
FROM pg_constraint
WHERE conname IN ('tasks_material_id_fkey', 'tasks_mode_id_fkey');

-- 이제 materials나 modes의 id를 변경하면
-- tasks 테이블의 material_id, mode_id가 자동으로 업데이트됩니다!
