-- ========================================
-- 1. Materials (재료) 테이블
-- ========================================
CREATE TABLE materials (
  id TEXT PRIMARY KEY,
  name TEXT NOT NULL,
  emoji TEXT NOT NULL,
  description TEXT,
  category TEXT NOT NULL,
  sizes JSONB NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- 초기 데이터 삽입
INSERT INTO materials (id, name, emoji, description, category, sizes) VALUES
('onion', '양파', '🧅', '신선한 양파로 다양한 요리를', 'vegetable', '{"small": 6, "medium": 8, "large": 10}'),
('potato', '감자', '🥔', '고소한 감자로 튀김 요리를', 'vegetable', '{"small": 5, "medium": 7, "large": 9}'),
('carrot', '당근', '🥕', '아삭한 당근으로 건강한 요리를', 'vegetable', '{"small": 4, "medium": 6, "large": 8}'),
('tomato', '토마토', '🍅', '신선한 토마토로 맛있는 요리를', 'vegetable', '{"small": 5, "medium": 7, "large": 9}');

-- ========================================
-- 2. Modes (손질 모드) 테이블
-- ========================================
CREATE TABLE modes (
  id TEXT PRIMARY KEY,
  name TEXT NOT NULL,
  icon TEXT NOT NULL,
  description TEXT,
  compatible_materials TEXT[] NOT NULL,
  duration INTEGER NOT NULL,
  difficulty TEXT NOT NULL,
  steps JSONB NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- 초기 데이터 삽입
INSERT INTO modes (id, name, icon, description, compatible_materials, duration, difficulty, steps) VALUES
('frying', '튀김', '🍤', '바삭하게 튀겨드립니다', ARRAY['onion', 'potato', 'carrot'], 180, 'medium',
  '[
    {"id": 1, "name": "재료 준비", "duration": 30, "description": "재료를 깨끗이 씻고 준비합니다"},
    {"id": 2, "name": "손질", "duration": 60, "description": "적당한 크기로 자릅니다"},
    {"id": 3, "name": "튀김", "duration": 90, "description": "180도 기름에 튀깁니다"}
  ]'::jsonb),
('slicing', '썰기', '🔪', '얇고 균일하게 썰어드립니다', ARRAY['onion', 'potato', 'carrot', 'tomato'], 120, 'easy',
  '[
    {"id": 1, "name": "재료 준비", "duration": 20, "description": "재료를 깨끗이 씻습니다"},
    {"id": 2, "name": "썰기", "duration": 100, "description": "얇고 균일하게 썰어냅니다"}
  ]'::jsonb),
('dicing', '깍둑썰기', '🎲', '정육면체 모양으로 썰어드립니다', ARRAY['onion', 'potato', 'carrot', 'tomato'], 150, 'medium',
  '[
    {"id": 1, "name": "재료 준비", "duration": 25, "description": "재료를 깨끗이 씻습니다"},
    {"id": 2, "name": "깍둑썰기", "duration": 125, "description": "정육면체 모양으로 썰어냅니다"}
  ]'::jsonb);

-- ========================================
-- 3. Tasks (작업 큐) 테이블
-- ========================================
CREATE TABLE tasks (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  material_id TEXT NOT NULL REFERENCES materials(id),
  mode_id TEXT NOT NULL REFERENCES modes(id),
  parameters JSONB NOT NULL,
  status TEXT NOT NULL DEFAULT 'pending' CHECK (status IN ('pending', 'running', 'paused', 'completed', 'failed')),
  priority INTEGER DEFAULT 1,
  progress INTEGER DEFAULT 0 CHECK (progress >= 0 AND progress <= 100),
  current_step INTEGER,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  started_at TIMESTAMP WITH TIME ZONE,
  completed_at TIMESTAMP WITH TIME ZONE,
  estimated_time INTEGER,
  error_message TEXT
);

-- 인덱스 생성 (성능 최적화)
CREATE INDEX idx_tasks_status ON tasks(status);
CREATE INDEX idx_tasks_created_at ON tasks(created_at DESC);

-- ========================================
-- 4. Robot State (로봇 상태) 테이블
-- ========================================
CREATE TABLE robot_state (
  id TEXT PRIMARY KEY DEFAULT 'current',
  status TEXT NOT NULL DEFAULT 'idle' CHECK (status IN ('idle', 'running', 'paused', 'error')),
  current_task_id UUID REFERENCES tasks(id),
  joint_states JSONB NOT NULL DEFAULT '{"position": [0,0,90,0,90,0], "velocity": [0,0,0,0,0,0], "effort": [0,0,0,0,0,0]}',
  tcp_position JSONB NOT NULL DEFAULT '{"x": 500.0, "y": 0.0, "z": 300.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}',
  error_state TEXT,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- 초기 로봇 상태 삽입
INSERT INTO robot_state (id) VALUES ('current');

-- ========================================
-- 5. Jobs (완료된 작업 히스토리) 테이블
-- ========================================
CREATE TABLE jobs (
  id UUID PRIMARY KEY,
  material_id TEXT NOT NULL,
  mode_id TEXT NOT NULL,
  parameters JSONB NOT NULL,
  status TEXT NOT NULL,
  progress INTEGER NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  started_at TIMESTAMP WITH TIME ZONE,
  completed_at TIMESTAMP WITH TIME ZONE NOT NULL,
  estimated_time INTEGER,
  actual_time INTEGER,
  error_message TEXT
);

CREATE INDEX idx_jobs_completed_at ON jobs(completed_at DESC);

-- ========================================
-- 6. Realtime 활성화
-- ========================================
-- Supabase Realtime 구독을 위한 설정
ALTER PUBLICATION supabase_realtime ADD TABLE tasks;
ALTER PUBLICATION supabase_realtime ADD TABLE robot_state;

-- ========================================
-- 7. Row Level Security (RLS) 설정
-- ========================================
-- RLS 활성화 (프로덕션에서 필수)
ALTER TABLE materials ENABLE ROW LEVEL SECURITY;
ALTER TABLE modes ENABLE ROW LEVEL SECURITY;
ALTER TABLE tasks ENABLE ROW LEVEL SECURITY;
ALTER TABLE robot_state ENABLE ROW LEVEL SECURITY;
ALTER TABLE jobs ENABLE ROW LEVEL SECURITY;

-- 모든 사용자에게 읽기 권한 (개발용)
CREATE POLICY "Allow public read" ON materials FOR SELECT USING (true);
CREATE POLICY "Allow public read" ON modes FOR SELECT USING (true);
CREATE POLICY "Allow public read" ON tasks FOR SELECT USING (true);
CREATE POLICY "Allow public read" ON robot_state FOR SELECT USING (true);
CREATE POLICY "Allow public read" ON jobs FOR SELECT USING (true);

-- 모든 사용자에게 쓰기 권한 (개발용 - 나중에 인증 추가 필요)
CREATE POLICY "Allow public insert" ON tasks FOR INSERT WITH CHECK (true);
CREATE POLICY "Allow public update" ON tasks FOR UPDATE USING (true);
CREATE POLICY "Allow public update" ON robot_state FOR UPDATE USING (true);
CREATE POLICY "Allow public insert" ON jobs FOR INSERT WITH CHECK (true);
