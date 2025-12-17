import { NextRequest, NextResponse } from 'next/server'
import { createClient } from '@supabase/supabase-js'

export async function POST(request: NextRequest) {
  try {
    const supabase = createClient(
      process.env.NEXT_PUBLIC_SUPABASE_URL!,
      process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY!
    )

    const body = await request.json()
    const { command } = body

    // Validate command
    const validCommands = ['pause', 'resume', 'stop', 'emergency_stop', 'home']
    if (!command || !validCommands.includes(command)) {
      return NextResponse.json(
        { success: false, error: 'Invalid command. Must be one of: pause, resume, stop, emergency_stop, home' },
        { status: 400 }
      )
    }

    // Prepare update data based on command
    const updateData: any = {
      desired_state: command,
      command_timestamp: new Date().toISOString()
    }

    // Emergency stop: set recovery_needed to true
    if (command === 'emergency_stop') {
      updateData.recovery_needed = true
    }

    // Update robot_state with the desired command
    const { data, error } = await supabase
      .from('robot_state')
      .update(updateData)
      .eq('id', 'current')
      .select()

    if (error) {
      console.error('Supabase error:', error)
      return NextResponse.json(
        { success: false, error: error.message },
        { status: 500 }
      )
    }

    return NextResponse.json({
      success: true,
      data: data,
      message: `Command '${command}' sent to robot`
    })
  } catch (error: any) {
    console.error('API error:', error)
    return NextResponse.json(
      { success: false, error: error.message },
      { status: 500 }
    )
  }
}
