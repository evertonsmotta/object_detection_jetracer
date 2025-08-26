#include "jetracer.hpp"
#include "mpc_integration_config.hpp"
#include "mpc_control_interface.hpp"
#include <stdexcept>
#include <thread>
#include <algorithm>
#include <cstring>
#include <cmath>

// Declarações das funções MPC (definidas em mpc_control_functions.cpp)
extern "C" {
    int isMPCEnabled();
    int isEmergencyStopActive();
}

namespace jetracer::control
{
	JetRacer::JetRacer(int servo_addr, int motor_addr)
		: servo_addr_(servo_addr),
		  motor_addr_(motor_addr),
		  running_(false),
		  servo_device_("/dev/i2c-1", servo_addr),
		  motor_device_("/dev/i2c-1", motor_addr)
	{
		init_servo();
		init_motors();
	}

	JetRacer::~JetRacer()
	{
		stop();
	}

	void JetRacer::init_servo()
	{
		try
		{
			servo_device_.write_byte(0x00, 0x06);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			servo_device_.write_byte(0x00, 0x10);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			servo_device_.write_byte(0xFE, 0x79);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			servo_device_.write_byte(0x01, 0x04);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			servo_device_.write_byte(0x00, 0x20);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		catch (const std::exception &e)
		{
			std::cerr << "Servo initialization failed: " << e.what() << std::endl;
			stop();
		}
	}

	void JetRacer::init_motors()
	{
		try
		{
			motor_device_.write_byte(0x00, 0x20);

			int prescale = static_cast<int>(std::floor(25000000.0 / 4096.0 / 100 - 1));
			int oldmode = motor_device_.read_byte(0x00);
			int newmode = (oldmode & 0x7F) | 0x10;

			motor_device_.write_byte(0x00, newmode);
			motor_device_.write_byte(0xFE, prescale);
			motor_device_.write_byte(0x00, oldmode);
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			motor_device_.write_byte(0x00, oldmode | 0xA1);
		}
		catch (const std::exception &e)
		{
			std::cerr << "Motor initialization failed: " << e.what() << std::endl;
			stop();
		}
	}

	void JetRacer::set_steering(int angle)
	{
		angle = std::max(-MAX_ANGLE_, std::min(angle, MAX_ANGLE_));

		// std::cout << "[DEBUG] set_steering = " << angle << std::endl;

		int pwm = 0;
		if (angle < 0)
		{
			std::cout << "Setting steering to left: " << angle << std::endl;
			pwm = SERVO_CENTER_PWM_ + (angle / static_cast<float>(MAX_ANGLE_)) * (SERVO_CENTER_PWM_ - SERVO_LEFT_PWM_);
		}
		else if (angle > 0)
		{
			pwm = SERVO_CENTER_PWM_ + (angle / static_cast<float>(MAX_ANGLE_)) * (SERVO_RIGHT_PWM_ - SERVO_CENTER_PWM_);
			std::cout << "Setting steering to right: " << angle << std::endl;
		}
		else
		{
			pwm = SERVO_CENTER_PWM_;
			std::cout << "Setting steering to center: " << angle << std::endl;
		}

		set_servo_pwm(0, 0, pwm);
		current_angle_ = angle;

		std::this_thread::sleep_for(std::chrono::milliseconds(servo_delay_ms_));
	}

	void JetRacer::smooth_steering(int target_angle, int increment)
	{
		// Se o angulo for um valor irrelevante, não faz nada
		// if (std::abs(target_angle) < 10)
		// {
		// 	std::cout << "Target angle " << target_angle << " is too small, skipping smooth steering." << std::endl;
		// 	return;
		// }
		target_angle = std::max(-MAX_ANGLE_, std::min(target_angle, MAX_ANGLE_));
		int step = (target_angle > current_angle_) ? increment : -increment;

		while ((step > 0 && current_angle_ < target_angle) || (step < 0 && current_angle_ > target_angle))
		{
			current_angle_ += step;
			if ((step > 0 && current_angle_ > target_angle) || (step < 0 && current_angle_ < target_angle))
			{
				current_angle_ = target_angle;
			}
			set_steering(current_angle_);
		}
	}

	void JetRacer::set_servo_pwm(int channel, int on_value, int off_value)
	{
		int base_reg = 0x06 + (channel * 4);
		servo_device_.write_byte(base_reg, on_value & 0xFF);
		servo_device_.write_byte(base_reg + 1, on_value >> 8);
		servo_device_.write_byte(base_reg + 2, off_value & 0xFF);
		servo_device_.write_byte(base_reg + 3, off_value >> 8);
	}

	void JetRacer::set_motor_pwm(int channel, int value)
	{
		value = std::max(0, std::min(value, 4095));
		int base_reg = 0x06 + (channel * 4);
		motor_device_.write_byte(base_reg, 0);
		motor_device_.write_byte(base_reg + 1, 0);
		motor_device_.write_byte(base_reg + 2, value & 0xFF);
		motor_device_.write_byte(base_reg + 3, value >> 8);
	}

	void JetRacer::set_speed(float speed, int angle)
	{
		//    std::cout << "[DEBUG] set_speed = " << speed << std::endl;

		if (speed > 25.0f)
			speed = 25.0f;

		speed = std::max(-100.0f, std::min(speed, 100.0f));
		int pwm_value = static_cast<int>(std::abs(speed) / 100.0f * 4095);

		if (speed > 0)
		{
			set_motor_pwm(0, pwm_value);
			set_motor_pwm(1, 0);
			set_motor_pwm(2, pwm_value);
			set_motor_pwm(5, pwm_value);
			set_motor_pwm(6, 0);
			set_motor_pwm(7, pwm_value);
		}
		else if (speed < 0)
		{
			set_motor_pwm(0, pwm_value);
			set_motor_pwm(1, pwm_value);
			set_motor_pwm(2, 0);
			set_motor_pwm(6, pwm_value);
			set_motor_pwm(7, pwm_value);
			set_motor_pwm(8, 0);
		}
		else
		{
			for (int channel = 0; channel < 9; ++channel)
			{
				set_motor_pwm(channel, 0);
			}
		}

		current_speed_ = speed;
	}

	void JetRacer::process_joystick()
	{
		if (SDL_Init(SDL_INIT_JOYSTICK) < 0)
		{
			std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
			return;
		}

		int joystick_index = -1;
		int num_joysticks = SDL_NumJoysticks();
		std::string desired_name = "SHANWAN Android Gamepad";

		std::cout << "SDL detected " << num_joysticks << " joystick(s):" << std::endl;

		for (int i = 0; i < num_joysticks; ++i)
		{
			const char *name = SDL_JoystickNameForIndex(i);
			std::cout << "  [" << i << "] " << name << std::endl;

			if (name && std::string(name).find(desired_name) != std::string::npos)
			{
				joystick_index = i;
				break;
			}
		}

		std::cout << "[INFO] Usando joystick: " << SDL_JoystickNameForIndex(joystick_index) << std::endl;

		if (joystick_index == -1)
		{
			std::cerr << "[ERRO] Nenhum joystick disponível!" << std::endl;
			SDL_Quit();
			return;
		}

		SDL_Joystick *joystick = SDL_JoystickOpen(joystick_index);
		if (!joystick)
		{
			std::cerr << "Failed to open joystick: " << SDL_GetError() << std::endl;
			SDL_Quit();
			return;
		}

		// Estados anteriores dos botões para detecção de borda
		bool r1_previous_state = false;
		bool r2_previous_state = false;
		bool l2_previous_state = false;
		bool l1_previous_state = false;

		while (running_)
		{
			SDL_JoystickUpdate();

			int left_joystick_y = SDL_JoystickGetAxis(joystick, JOYSTICK_ACCELERATOR_AXIS);	 // acelerador
			int right_joystick_x = SDL_JoystickGetAxis(joystick, JOYSTICK_STEERING_AXIS); // direção
			int r1_button = SDL_JoystickGetButton(joystick, JOYSTICK_ASSISTANCE_MODE_BUTTON);	 // R1 - Modo assistência
			int r2_button = SDL_JoystickGetButton(joystick, JOYSTICK_AUTONOMOUS_MODE_BUTTON);	 // R2 - Modo autônomo
			int l2_button = SDL_JoystickGetButton(joystick, JOYSTICK_EMERGENCY_BUTTON);		 // L2 - Parada de emergência
			int l1_button = SDL_JoystickGetButton(joystick, JOYSTICK_RESET_BUTTON);	 // L1 - Reset

			// Alternar para MODO ASSISTÊNCIA via R1
			if (r1_button == 1 && !r1_previous_state)
			{
				current_mode_ = OperationMode::ASSISTANCE;
				std::cout << "[JetRacer] Modo ASSISTÊNCIA ativado - Velocidade manual + Direção automática" << std::endl;

				// Sincronizar com o sistema MPC para modo de assistência
				toggleMPCViaJoystick();

				// Se entrando em modo de assistência, parar o carro primeiro
				set_speed(0, 0);
				std::cout << "[JetRacer] Modo assistência de direção ativado" << std::endl;
			}
			r1_previous_state = r1_button;

			// MODO AUTÔNOMO TEMPORARIAMENTE DESABILITADO PARA TESTE
			if (r2_button == 1 && !r2_previous_state)
			{
				std::cout << "[JetRacer] ⚠️  MODO AUTÔNOMO TEMPORARIAMENTE DESABILITADO PARA TESTE" << std::endl;
				std::cout << "[JetRacer] 📋 Use apenas: MODO MANUAL (padrão) e MODO ASSISTÊNCIA (R1)" << std::endl;
				std::cout << "[JetRacer] 🔄 Para reativar: Descomente o código no arquivo jetracer.cpp" << std::endl;

				// CÓDIGO COMENTADO TEMPORARIAMENTE:
				// current_mode_ = OperationMode::AUTONOMOUS;
				// toggleMPCViaJoystick();
				// set_speed(0, 0);
			}
			r2_previous_state = r2_button;

			// Parada de emergência via L2
			if (l2_button == 1 && !l2_previous_state)
			{
				std::cout << "Emergency stop activated via joystick" << std::endl;

				// Sincronizar com o sistema MPC
				emergencyStopViaJoystick();
			}
			l2_previous_state = l2_button;

			// Reset via L1
			if (l1_button == 1 && !l1_previous_state)
			{
				std::cout << "Reset activated via joystick" << std::endl;

				// Sincronizar com o sistema MPC
				resetViaJoystick();
			}
			l1_previous_state = l1_button;

			// Lógica dos três modos de operação
			switch (current_mode_)
			{
				case OperationMode::MANUAL:
				{
					// MODO MANUAL: Controle total via joystick
					float speed = -left_joystick_y / 32767.0f * MAX_SPEED;
					float angle = right_joystick_x / 32767.0f * MAX_ANGLE;

					set_speed(speed, angle);
					smooth_steering(angle, 10);
					break;
				}

				case OperationMode::ASSISTANCE:
				{
					// MODO ASSISTÊNCIA: Velocidade manual + direção automática
					float manual_speed = -left_joystick_y / 32767.0f * MAX_SPEED;

					// Armazena velocidade manual para o sistema MPC usar
					manual_speed_for_assist_ = manual_speed;

					// Aplica apenas a velocidade (direção será controlada pelo MPC)
					set_speed(manual_speed, current_angle_);

					// Verificar se o usuário acionou o joystick direito para voltar ao modo manual
					if (std::abs(right_joystick_x) > 1000) // Threshold para detectar movimento do joystick
					{
						current_mode_ = OperationMode::MANUAL;
						std::cout << "[JetRacer] Modo MANUAL ativado via joystick direito" << std::endl;
					}

					static int log_counter = 0;
					if ((++log_counter % 100) == 0) { // Log a cada ~5 segundos (50ms * 100)
						std::cout << "[JetRacer] Assistência ativa. Speed: " << manual_speed
								  << "%, MPC Status: " << (isMPCEnabled() ? "ENABLED" : "DISABLED")
								  << ", Emergency: " << (isEmergencyStopActive() ? "ACTIVE" : "INACTIVE")
								  << std::endl;
					}
					break;
				}

				case OperationMode::AUTONOMOUS:
				{
					// MODO AUTÔNOMO TEMPORARIAMENTE DESABILITADO
					std::cout << "[JetRacer] ⚠️  MODO AUTÔNOMO DESABILITADO - Voltando para MANUAL" << std::endl;
					current_mode_ = OperationMode::MANUAL;
					break;
				}
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(1000 / CONTROL_UPDATE_RATE));
		}

		SDL_JoystickClose(joystick);
		SDL_Quit();
	}

	void JetRacer::start()
	{
		running_ = true;
		std::thread joystick_thread(&JetRacer::process_joystick, this);
		joystick_thread.detach();
	}

	void JetRacer::stop()
	{
		running_ = false;
		set_speed(0, 0);
		set_steering(0);
	}

	bool JetRacer::is_running() const
	{
		return running_.load();
	}

} // namespace jetracer::control
