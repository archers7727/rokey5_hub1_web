import { NextResponse } from 'next/server'
import { createClient } from '@supabase/supabase-js'

// GET /api/modes?material=onion
export async function GET(request: Request) {
  const { searchParams } = new URL(request.url)
  const material = searchParams.get('material')

  const supabase = createClient(
    process.env.NEXT_PUBLIC_SUPABASE_URL!,
    process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY!
  )

  let query = supabase
    .from('modes')
    .select('*')
    .order('name')

  // Filter by material if provided
  if (material) {
    query = query.contains('compatible_materials', [material])
  }

  const { data: modes, error } = await query

  if (error) {
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }

  return NextResponse.json({ success: true, data: modes })
}
