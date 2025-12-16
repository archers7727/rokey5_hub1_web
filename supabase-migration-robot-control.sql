-- ========================================
-- Robot State Table Migration
-- Add columns for Doosan robot state and command control
-- ========================================

-- Add new columns to robot_state table
ALTER TABLE robot_state
ADD COLUMN IF NOT EXISTS doosan_robot_state INTEGER DEFAULT 1,
ADD COLUMN IF NOT EXISTS desired_state TEXT,
ADD COLUMN IF NOT EXISTS command_timestamp TIMESTAMP WITH TIME ZONE;

-- Add comment for doosan_robot_state values
COMMENT ON COLUMN robot_state.doosan_robot_state IS
'Doosan robot state: 0=INITIALIZING, 1=STANDBY, 2=MOVING, 3=SAFE_OFF, 4=TEACHING, 5=SAFE_STOP, 6=EMERGENCY_STOP, 7=HOMMING, 8=RECOVERY, 9=SAFE_STOP2, 10=SAFE_OFF2, 11-14=RESERVED, 15=NOT_READY';

COMMENT ON COLUMN robot_state.desired_state IS
'Command from web UI: pause, resume, stop, emergency_stop, etc.';

COMMENT ON COLUMN robot_state.command_timestamp IS
'Timestamp when command was issued from web UI';
