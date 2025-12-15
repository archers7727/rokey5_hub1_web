import { NextResponse } from 'next/server'
import { createClient } from '@/lib/supabase/server'

// GET /api/jobs
export async function GET() {
  const supabase = await createClient()

  const { data: jobs, error } = await supabase
    .from('jobs')
    .select('*')
    .order('completed_at', { ascending: false })
    .limit(50)

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({ success: true, data: jobs })
}

// POST /api/jobs
export async function POST(request: Request) {
  const body = await request.json()
  const supabase = await createClient()

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
