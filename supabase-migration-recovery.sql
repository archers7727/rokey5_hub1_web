-- Migration: Add recovery_needed column for emergency stop recovery flow
-- Created: 2025-12-16

-- Add recovery_needed column to track if robot needs recovery after emergency stop
ALTER TABLE robot_state
ADD COLUMN IF NOT EXISTS recovery_needed BOOLEAN DEFAULT false;

-- Add comment for documentation
COMMENT ON COLUMN robot_state.recovery_needed IS
'Indicates if robot needs recovery after emergency stop. Set to true when emergency_stop command is executed, set to false when robot reaches home position.';

-- Set default value for existing row
UPDATE robot_state
SET recovery_needed = false
WHERE id = 'current';
