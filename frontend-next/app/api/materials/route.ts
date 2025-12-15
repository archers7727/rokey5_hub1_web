import { NextResponse } from 'next/server'
import { createClient } from '@/lib/supabase/server'

// GET /api/materials
export async function GET() {
  const supabase = await createClient()

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
