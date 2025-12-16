-- ========================================
-- Complete Robot State Table Migration
-- Combines all robot control and recovery features
-- ========================================

-- 1. Add Doosan robot state columns (if not exists)
ALTER TABLE robot_state
ADD COLUMN IF NOT EXISTS doosan_robot_state INTEGER DEFAULT 1,
ADD COLUMN IF NOT EXISTS desired_state TEXT,
ADD COLUMN IF NOT EXISTS command_timestamp TIMESTAMP WITH TIME ZONE;

-- 2. Add recovery column for emergency stop recovery flow
ALTER TABLE robot_state
ADD COLUMN IF NOT EXISTS recovery_needed BOOLEAN DEFAULT false;

-- 3. Add comments for documentation
COMMENT ON COLUMN robot_state.doosan_robot_state IS
'Doosan robot state: 0=INITIALIZING, 1=STANDBY, 2=MOVING, 3=SAFE_OFF, 4=TEACHING, 5=SAFE_STOP, 6=EMERGENCY_STOP, 7=HOMMING, 8=RECOVERY, 9=SAFE_STOP2, 10=SAFE_OFF2, 11-14=RESERVED, 15=NOT_READY';

COMMENT ON COLUMN robot_state.desired_state IS
'Command from web UI: pause, resume, stop, emergency_stop, move_to_home';

COMMENT ON COLUMN robot_state.command_timestamp IS
'Timestamp when command was issued from web UI';

COMMENT ON COLUMN robot_state.recovery_needed IS
'Indicates if robot needs recovery after emergency stop. Set to true when emergency_stop command is executed, set to false when robot reaches home position.';

-- 4. Set default values for existing row
UPDATE robot_state
SET
  doosan_robot_state = COALESCE(doosan_robot_state, 1),
  recovery_needed = COALESCE(recovery_needed, false)
WHERE id = 'current';

-- 5. Verify the migration
SELECT
  column_name,
  data_type,
  column_default,
  is_nullable
FROM information_schema.columns
WHERE table_name = 'robot_state'
ORDER BY ordinal_position;
