interface ProgressIndicatorProps {
  current: number
  total: number
}

export const ProgressIndicator = ({ current, total }: ProgressIndicatorProps) => {
  return (
    <div className="flex items-center space-x-2">
      {Array.from({ length: total }, (_, i) => i + 1).map((step) => (
        <div
          key={step}
          className={`flex items-center ${step < total ? 'mr-2' : ''}`}
        >
          <div
            className={`w-8 h-8 rounded-full flex items-center justify-center text-sm font-semibold transition-colors ${
              step <= current
                ? 'bg-blue-600 text-white'
                : 'bg-gray-200 text-gray-600'
            }`}
          >
            {step}
          </div>
          {step < total && (
            <div
              className={`w-8 h-0.5 ${
                step < current ? 'bg-blue-600' : 'bg-gray-200'
              }`}
            />
          )}
        </div>
      ))}
    </div>
  )
}
