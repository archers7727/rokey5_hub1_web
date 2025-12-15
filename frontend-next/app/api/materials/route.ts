import { NextResponse } from 'next/server'
import { createClient } from '@supabase/supabase-js'

// GET /api/materials
export async function GET() {
  const supabase = createClient(
    process.env.NEXT_PUBLIC_SUPABASE_URL!,
    process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY!
  )

  const { data: materials, error } = await supabase
    .from('materials')
    .select('*')
    .order('name')

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({ success: true, data: materials })
}
