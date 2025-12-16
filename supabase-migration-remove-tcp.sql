-- ========================================
-- Remove TCP Position Column Migration
-- Remove tcp_position column as it's not used
-- ========================================

-- Remove tcp_position column from robot_state table
ALTER TABLE robot_state DROP COLUMN IF EXISTS tcp_position;
