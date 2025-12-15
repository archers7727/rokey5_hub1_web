import { NextResponse } from 'next/server'
import { createClient } from '@supabase/supabase-js'

// GET /api/robot/state
export async function GET() {
  const supabase = createClient(
    process.env.NEXT_PUBLIC_SUPABASE_URL!,
    process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY!
  )

  const { data: state, error } = await supabase
    .from('robot_state')
    .select('*')
    .eq('id', 'current')
    .single()

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({ success: true, data: state })
}
