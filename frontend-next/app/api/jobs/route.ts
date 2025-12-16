import { NextRequest, NextResponse } from 'next/server'
import { createClient } from '@supabase/supabase-js'

// GET /api/jobs - tasks 테이블 조회로 변경
export async function GET(request: NextRequest) {
  const supabase = createClient(
    process.env.NEXT_PUBLIC_SUPABASE_URL!,
    process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY!
  )

  // task_id 파라미터 확인
  const searchParams = request.nextUrl.searchParams
  const taskId = searchParams.get('task_id')

  let query = supabase.from('tasks').select('*')

  // task_id가 있으면 특정 task 조회
  if (taskId) {
    query = query.eq('id', taskId)
  } else {
    // 없으면 최근 작업 조회
    query = query.order('created_at', { ascending: false }).limit(50)
  }

  const { data: tasks, error } = await query

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({ success: true, data: tasks })
}

// POST /api/jobs
export async function POST(request: Request) {
  const body = await request.json()

  const supabase = createClient(
    process.env.NEXT_PUBLIC_SUPABASE_URL!,
    process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY!
  )

  // Add new task to tasks table
  const { data: task, error } = await supabase
    .from('tasks')
    .insert({
      material_id: body.material,
      mode_id: body.mode,
      parameters: body.parameters,
      status: 'pending',
      priority: 1,
      progress: 0,
      estimated_time: body.estimatedTime || 0,
    })
    .select()
    .single()

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({
    success: true,
    data: task,
    message: 'Task queued successfully'
  })
}
