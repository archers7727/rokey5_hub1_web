import { createClient as createSupabaseClient } from '@supabase/supabase-js'

export function createClient() {
  const supabaseUrl = process.env.NEXT_PUBLIC_SUPABASE_URL || ''
  const supabaseAnonKey = process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY || ''

  // Return a dummy client during build if env vars aren't available
  if (!supabaseUrl || !supabaseAnonKey) {
    return createSupabaseClient('https://placeholder.supabase.co', 'placeholder-key')
  }

  return createSupabaseClient(supabaseUrl, supabaseAnonKey)
}
