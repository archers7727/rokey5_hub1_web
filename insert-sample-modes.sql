-- ========================================
-- Insert sample modes data
-- ========================================

INSERT INTO modes (id, name, icon, description, duration, compatible_materials)
VALUES
  ('slice', '슬라이스', '🔪', '얇게 자르기', 40, ARRAY['carret']),
  ('dice', '다이스', '🎲', '깍둑 썰기', 45, ARRAY['carret']),
  ('julienne', '채썰기', '✂️', '가늘게 채썰기', 50, ARRAY['carret'])
ON CONFLICT (id) DO UPDATE SET
  name = EXCLUDED.name,
  icon = EXCLUDED.icon,
  description = EXCLUDED.description,
  duration = EXCLUDED.duration,
  compatible_materials = EXCLUDED.compatible_materials;

-- Verify the data
SELECT * FROM modes ORDER BY name;
