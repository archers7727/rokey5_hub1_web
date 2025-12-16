-- ========================================
-- Check robot_state table schema
-- Run this in Supabase SQL Editor to verify columns
-- ========================================

-- Check all columns in robot_state table
SELECT
  column_name,
  data_type,
  column_default,
  is_nullable
FROM information_schema.columns
WHERE table_name = 'robot_state'
ORDER BY ordinal_position;

-- Check current robot_state data
SELECT * FROM robot_state WHERE id = 'current';
